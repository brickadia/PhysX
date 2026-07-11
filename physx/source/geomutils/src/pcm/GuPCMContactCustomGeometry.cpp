// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geomutils/PxContactBuffer.h"
#include "foundation/PxSort.h"
#include "GuVecBox.h"
#include "GuBounds.h"
#include "GuContactMethodImpl.h"
#include "GuPCMShapeConvex.h"
#include "GuPCMContactGen.h"
#include "GuPCMContactConvexCommon.h"

using namespace physx;
using namespace aos;
using namespace Gu;

namespace
{
struct ContactReceiverImpl : PxCustomGeometry::Callbacks::ContactReceiver
{
	static constexpr PxU32 MAX_MANIFOLD_CONTACTS = PxContactBuffer::MAX_CONTACTS;

	MeshPersistentContact mManifoldContacts[MAX_MANIFOLD_CONTACTS];
	PCMContactPatch mContactPatch[PCM_MAX_CONTACTPATCH_SIZE];
	PCMContactPatch* mContactPatchPtr[PCM_MAX_CONTACTPATCH_SIZE];
	PxU32 mNumContacts;
	PxU32 mNumContactPatch;
	MultiplePersistentContactManifold& mMultiManifold;
	const PxTransformV& mTransf0;
	const PxTransformV& mTransf1;
	FloatV mSqReplaceBreakingThreshold;
	FloatV mAcceptanceEpsilon;
	FloatV mPlaneTolerance;

	ContactReceiverImpl(MultiplePersistentContactManifold& multiManifold,
		const PxTransformV& transf0, const PxTransformV& transf1,
		const FloatV& replaceBreakingThreshold, PxReal toleranceLength)
		: mNumContacts(0)
		, mNumContactPatch(0)
		, mMultiManifold(multiManifold)
		, mTransf0(transf0)
		, mTransf1(transf1)
	{
		mSqReplaceBreakingThreshold = FMul(replaceBreakingThreshold, replaceBreakingThreshold);
		mAcceptanceEpsilon = FLoad(0.996f);
		mPlaneTolerance = FLoad(toleranceLength * 0.0075f);

		for(PxU32 i = 0; i < PCM_MAX_CONTACTPATCH_SIZE; ++i)
			mContactPatchPtr[i] = &mContactPatch[i];
	}

	static constexpr PxU32 MAX_REDUCE_CONTACTS = MAX_MANIFOLD_CONTACTS + GU_SINGLE_MANIFOLD_CACHE_SIZE;

	// Deepest contact plus the convex hull of the patch footprint, so every outer corner survives reduction.
	PxU32 reduceBatchPreservingBoundary(MeshPersistentContact* contacts, PxU32 count, const Vec3VArg planeNormal, PxU32 budget) const
	{
		PX_ASSERT(count <= MAX_REDUCE_CONTACTS);
		PX_ASSERT(budget >= 2 && budget <= 32);

		PxVec3 normal;
		V3StoreU(planeNormal, normal);

		PxVec3 tangent0 = PxAbs(normal.x) < 0.707f ? PxVec3(1.f, 0.f, 0.f).cross(normal) : PxVec3(0.f, 1.f, 0.f).cross(normal);
		tangent0.normalize();
		const PxVec3 tangent1 = normal.cross(tangent0);

		float px[MAX_REDUCE_CONTACTS];
		float py[MAX_REDUCE_CONTACTS];
		float pens[MAX_REDUCE_CONTACTS];
		bool chosen[MAX_REDUCE_CONTACTS];

		for(PxU32 i = 0; i < count; ++i)
		{
			PxVec3 p;
			V3StoreU(contacts[i].mLocalPointB, p);
			px[i] = p.dot(tangent0);
			py[i] = p.dot(tangent1);
			FStore(V4GetW(contacts[i].mLocalNormalPen), &pens[i]);
			chosen[i] = false;
		}

		PxU32 deepest = 0;
		for(PxU32 i = 1; i < count; ++i)
		{
			if(pens[i] < pens[deepest])
				deepest = i;
		}
		chosen[deepest] = true;
		PxU32 numChosen = 1;

		// Monotone chain convex hull; popping collinear points keeps true corners only
		PxU32 order[MAX_REDUCE_CONTACTS];
		for(PxU32 i = 0; i < count; ++i)
			order[i] = i;

		PxSort(order, count, [&](PxU32 a, PxU32 b) { return px[a] < px[b] || (px[a] == px[b] && py[a] < py[b]); });

		const auto cross2 = [&](PxU32 o, PxU32 a, PxU32 b) -> float
		{
			return (px[a] - px[o]) * (py[b] - py[o]) - (py[a] - py[o]) * (px[b] - px[o]);
		};

		PxU32 hull[MAX_REDUCE_CONTACTS * 2];
		PxU32 numHull = 0;

		for(PxU32 i = 0; i < count; ++i)
		{
			const PxU32 idx = order[i];
			while(numHull >= 2 && cross2(hull[numHull - 2], hull[numHull - 1], idx) <= 0.f)
				numHull--;
			hull[numHull++] = idx;
		}

		const PxU32 lowerEnd = numHull + 1;
		for(PxI32 i = PxI32(count) - 2; i >= 0; --i)
		{
			const PxU32 idx = order[PxU32(i)];
			while(numHull >= lowerEnd && cross2(hull[numHull - 2], hull[numHull - 1], idx) <= 0.f)
				numHull--;
			hull[numHull++] = idx;
		}
		numHull--;	// last hull point duplicates the first

		// The deepest is usually itself a hull corner; only an interior deepest costs the hull a slot
		PxU32 hullChosen = 0;
		for(PxU32 i = 0; i < numHull; ++i)
			hullChosen += chosen[hull[i]] ? 1u : 0u;

		if(numHull - hullChosen <= budget - numChosen)
		{
			for(PxU32 i = 0; i < numHull; ++i)
			{
				const PxU32 idx = hull[i];
				if(!chosen[idx])
				{
					chosen[idx] = true;
					numChosen++;
				}
			}
		}
		else
		{
			// Hull bigger than the budget: farthest point sampling keeps the extremes evenly spaced
			float minDist[MAX_REDUCE_CONTACTS];
			PxU32 last = 0xffffffff;

			for(PxU32 i = 0; i < numHull; ++i)
			{
				minDist[i] = PX_MAX_F32;
				if(chosen[hull[i]])
					last = hull[i];
			}

			if(last == 0xffffffff)
			{
				last = hull[0];
				chosen[last] = true;
				numChosen++;
			}

			while(numChosen < budget)
			{
				PxU32 best = 0xffffffff;
				float bestDist = -1.f;

				for(PxU32 i = 0; i < numHull; ++i)
				{
					const PxU32 idx = hull[i];
					const float d = (px[idx] - px[last]) * (px[idx] - px[last]) + (py[idx] - py[last]) * (py[idx] - py[last]);
					minDist[i] = PxMin(minDist[i], d);

					if(!chosen[idx] && minDist[i] > bestDist)
					{
						bestDist = minDist[i];
						best = idx;
					}
				}

				if(best == 0xffffffff)
					break;

				chosen[best] = true;
				numChosen++;
				last = best;
			}
		}

		while(numChosen < budget)
		{
			PxU32 best = 0xffffffff;
			for(PxU32 i = 0; i < count; ++i)
			{
				if(!chosen[i] && (best == 0xffffffff || pens[i] < pens[best]))
					best = i;
			}

			if(best == 0xffffffff)
				break;

			chosen[best] = true;
			numChosen++;
		}

		PxU32 write = 0;
		for(PxU32 i = 0; i < count; ++i)
		{
			if(chosen[i])
				contacts[write++] = contacts[i];
		}
		return write;
	}

	virtual bool reportContacts(const PxContactPoint* contacts, PxU32 numContacts, const PxVec3& patchNormal) override
	{
		if(numContacts == 0)
			return true;

		const PxU32 previousNumContacts = mNumContacts;

		for(PxU32 i = 0; i < numContacts && mNumContacts < MAX_MANIFOLD_CONTACTS; ++i)
		{
			const Vec3V worldPoint = V3LoadU(contacts[i].point);
			const Vec3V worldNormal = V3LoadU(contacts[i].normal);
			const FloatV separation = FLoad(contacts[i].separation);

			// Offset localPointA along the contact normal by the separation so that
			// refreshContactPoints (which recomputes separation as dot(transformedA - B, normal))
			// recovers the correct separation instead of collapsing to ~0.
			const Vec3V worldPointA = V3ScaleAdd(worldNormal, separation, worldPoint);

			mManifoldContacts[mNumContacts].mLocalPointA = mTransf0.transformInv(worldPointA);
			mManifoldContacts[mNumContacts].mLocalPointB = mTransf1.transformInv(worldPoint);
			mManifoldContacts[mNumContacts].mLocalNormalPen = V4SetW(V4Zero(), separation);
			mManifoldContacts[mNumContacts].mFaceIndex = contacts[i].internalFaceIndex1;
			mNumContacts++;
		}

		if(mNumContacts == previousNumContacts)
			return true;

		const PxU32 newContacts = mNumContacts - previousNumContacts;

		if(newContacts > GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE)
		{
			const Vec3V localNormal = mTransf1.rotateInv(V3LoadU(patchNormal));
			mNumContacts = previousNumContacts + reduceBatchPreservingBoundary(&mManifoldContacts[previousNumContacts], newContacts, localNormal, GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE);
		}

		for(PxU32 i = previousNumContacts; i < mNumContacts; ++i)
		{
			for(PxU32 j = i + 1; j < mNumContacts; ++j)
			{
				const Vec3V dif = V3Sub(mManifoldContacts[j].mLocalPointB, mManifoldContacts[i].mLocalPointB);
				const FloatV d = V3Dot(dif, dif);
				if(FAllGrtr(mSqReplaceBreakingThreshold, d))
				{
					// Keep the deeper contact, matching the patch-merge dedup below.
					if(FAllGrtr(V4GetW(mManifoldContacts[i].mLocalNormalPen), V4GetW(mManifoldContacts[j].mLocalNormalPen)))
						mManifoldContacts[i] = mManifoldContacts[j];
					mManifoldContacts[j] = mManifoldContacts[mNumContacts - 1];
					mNumContacts--;
					j--;
				}
			}
		}

		const Vec3V localPatchNormal = mTransf1.rotateInv(V3LoadU(patchNormal));

		FloatV maxPen = FMax();
		for(PxU32 i = previousNumContacts; i < mNumContacts; ++i)
		{
			const FloatV pen = V4GetW(mManifoldContacts[i].mLocalNormalPen);
			mManifoldContacts[i].mLocalNormalPen = V4SetW(localPatchNormal, pen);
			maxPen = FMin(maxPen, pen);
		}

		bool foundPatch = false;
		if(mNumContactPatch > 0)
		{
			// Same-normal patches on distinct parallel planes (stacked brick layers) must stay separate.
			if(FAllGrtr(V3Dot(mContactPatch[mNumContactPatch - 1].mPatchNormal, localPatchNormal), mAcceptanceEpsilon)
				&& FAllGrtr(mPlaneTolerance, FAbs(FSub(mContactPatch[mNumContactPatch - 1].mPatchMaxPen, maxPen))))
			{
				PCMContactPatch& patch = mContactPatch[mNumContactPatch - 1];

				for(PxU32 i = patch.mStartIndex; i < patch.mEndIndex; ++i)
				{
					for(PxU32 j = previousNumContacts; j < mNumContacts; ++j)
					{
						const Vec3V dif = V3Sub(mManifoldContacts[j].mLocalPointB, mManifoldContacts[i].mLocalPointB);
						const FloatV d = V3Dot(dif, dif);
						if(FAllGrtr(mSqReplaceBreakingThreshold, d))
						{
							if(FAllGrtr(V4GetW(mManifoldContacts[i].mLocalNormalPen), V4GetW(mManifoldContacts[j].mLocalNormalPen)))
								mManifoldContacts[i] = mManifoldContacts[j];
							mManifoldContacts[j] = mManifoldContacts[mNumContacts - 1];
							mNumContacts--;
							j--;
						}
					}
				}

				patch.mEndIndex = mNumContacts;
				patch.mPatchMaxPen = FMin(patch.mPatchMaxPen, maxPen);
				foundPatch = true;
			}
		}

		if(!foundPatch && mNumContactPatch < PCM_MAX_CONTACTPATCH_SIZE)
		{
			mContactPatch[mNumContactPatch].mStartIndex = previousNumContacts;
			mContactPatch[mNumContactPatch].mEndIndex = mNumContacts;
			mContactPatch[mNumContactPatch].mPatchMaxPen = maxPen;
			mContactPatch[mNumContactPatch].mPatchNormal = localPatchNormal;
			mNumContactPatch++;
		}

		PX_ASSERT(mNumContacts <= MAX_MANIFOLD_CONTACTS);
		if(mNumContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
			processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, true);

		return mNumContacts < MAX_MANIFOLD_CONTACTS;
	}

	// refineContactPatchConnective with an added coplanarity requirement, so parallel planes keep their own manifolds.
	void refinePatchesCoplanar()
	{
		for(PxU32 i = 0; i < mNumContactPatch; ++i)
		{
			PCMContactPatch* patch = mContactPatchPtr[i];
			patch->mRoot = patch;
			patch->mEndPatch = patch;
			patch->mTotalSize = patch->mEndIndex - patch->mStartIndex;
			patch->mNextPatch = NULL;

			for(PxU32 j = i; j > 0; --j)
			{
				PCMContactPatch* other = mContactPatchPtr[j - 1];
				const FloatV d = V3Dot(patch->mPatchNormal, other->mRoot->mPatchNormal);
				if(FAllGrtrOrEq(d, mAcceptanceEpsilon)
					&& FAllGrtr(mPlaneTolerance, FAbs(FSub(patch->mPatchMaxPen, other->mRoot->mPatchMaxPen))))
				{
					other->mNextPatch = patch;
					other->mRoot->mEndPatch = patch;
					patch->mRoot = other->mRoot;
					other->mRoot->mTotalSize += patch->mEndIndex - patch->mStartIndex;
					break;
				}
			}
		}
	}

	// Plane-aware replacement for addManifoldContactPoints, whose incremental branch matches by normal only and collapses parallel planes.
	void assignPatchesToManifolds(PxU32 maxContactsPerManifold)
	{
		for(PxU32 i = 0; i < mNumContactPatch; ++i)
		{
			PCMContactPatch* patch = mContactPatchPtr[i];

			if(patch->mRoot != patch)
				continue;

			MeshPersistentContact gathered[MAX_REDUCE_CONTACTS];
			PxU32 numGathered = 0;

			for(const PCMContactPatch* current = patch; current; current = current->mNextPatch)
			{
				for(PxU32 k = current->mStartIndex; k < current->mEndIndex && numGathered < MAX_MANIFOLD_CONTACTS; ++k)
					gathered[numGathered++] = mManifoldContacts[k];
			}

			if(numGathered == 0)
				continue;

			SinglePersistentContactManifold* target = NULL;
			PxU32 targetIndex = 0;
			SinglePersistentContactManifold* sameNormal = NULL;
			PxU32 sameNormalIndex = 0;

			for(PxU32 j = 0; j < mMultiManifold.mNumManifolds; ++j)
			{
				SinglePersistentContactManifold& manifold = *mMultiManifold.getManifold(j);

				if(manifold.mNumContacts == 0)
					continue;

				if(FAllGrtr(mAcceptanceEpsilon, V3Dot(patch->mPatchNormal, manifold.getLocalNormal())))
					continue;

				if(!sameNormal)
				{
					sameNormal = &manifold;
					sameNormalIndex = j;
				}

				if(FAllGrtr(mPlaneTolerance, FAbs(FSub(patch->mPatchMaxPen, FLoad(mMultiManifold.mMaxPen[mMultiManifold.mManifoldIndices[j]])))))
				{
					target = &manifold;
					targetIndex = j;
					break;
				}
			}

			bool bIsFresh = false;
			bool bMixedPlanes = false;

			if(!target)
			{
				if(SinglePersistentContactManifold* fresh = mMultiManifold.getEmptyManifold())
				{
					fresh->mNumContacts = 0;
					target = fresh;
					bIsFresh = true;
				}
				else if(sameNormal)
				{
					target = sameNormal;
					targetIndex = sameNormalIndex;
					bMixedPlanes = true;
				}
				else
				{
					// Patches are depth sorted, so only the shallowest are dropped
					continue;
				}
			}

			for(PxU32 k = 0; k < target->mNumContacts && numGathered < MAX_REDUCE_CONTACTS; ++k)
				gathered[numGathered++] = target->mContactPoints[k];

			FloatV minPen = V4GetW(gathered[0].mLocalNormalPen);

			for(PxU32 k = 1; k < numGathered; ++k)
				minPen = FMin(minPen, V4GetW(gathered[k].mLocalNormalPen));

			// Near-plane-wins only for the slot-exhaustion merge; a tilted footprint on one plane spans more than mPlaneTolerance.
			if(bMixedPlanes)
			{
				const FloatV penLimit = FAdd(minPen, mPlaneTolerance);
				PxU32 write = 0;

				for(PxU32 k = 0; k < numGathered; ++k)
				{
					if(FAllGrtrOrEq(penLimit, V4GetW(gathered[k].mLocalNormalPen)))
						gathered[write++] = gathered[k];
				}
				numGathered = write;
			}

			for(PxU32 a = 0; a < numGathered; ++a)
			{
				for(PxU32 b = a + 1; b < numGathered; ++b)
				{
					const Vec3V dif = V3Sub(gathered[b].mLocalPointB, gathered[a].mLocalPointB);

					if(FAllGrtr(mSqReplaceBreakingThreshold, V3Dot(dif, dif)))
					{
						if(FAllGrtr(V4GetW(gathered[a].mLocalNormalPen), V4GetW(gathered[b].mLocalNormalPen)))
							gathered[a] = gathered[b];
						gathered[b] = gathered[--numGathered];
						b--;
					}
				}
			}

			if(numGathered > maxContactsPerManifold)
				numGathered = reduceBatchPreservingBoundary(gathered, numGathered, patch->mPatchNormal, maxContactsPerManifold);

			for(PxU32 k = 0; k < numGathered; ++k)
				target->mContactPoints[k] = gathered[k];
			target->mNumContacts = numGathered;

			if(bIsFresh)
			{
				FStore(minPen, &mMultiManifold.mMaxPen[mMultiManifold.mManifoldIndices[mMultiManifold.mNumManifolds]]);
				mMultiManifold.mNumManifolds++;
			}
			else
			{
				FStore(minPen, &mMultiManifold.mMaxPen[mMultiManifold.mManifoldIndices[targetIndex]]);
			}
		}
	}

	void processContacts(PxU8 maxContactsPerManifold, bool isNotLastPatch)
	{
		if(mNumContacts == 0)
			return;

		for(PxU32 i = 1; i < mNumContactPatch; ++i)
		{
			const PxU32 indexi = i - 1;
			if(FAllGrtr(mContactPatchPtr[indexi]->mPatchMaxPen, mContactPatchPtr[i]->mPatchMaxPen))
			{
				PCMContactPatch* tmp = mContactPatchPtr[indexi];
				mContactPatchPtr[indexi] = mContactPatchPtr[i];
				mContactPatchPtr[i] = tmp;

				for(PxI32 j = PxI32(i - 2); j >= 0; j--)
				{
					const PxU32 indexj = PxU32(j + 1);
					if(FAllGrtrOrEq(mContactPatchPtr[indexj]->mPatchMaxPen, mContactPatchPtr[j]->mPatchMaxPen))
						break;
					PCMContactPatch* temp = mContactPatchPtr[indexj];
					mContactPatchPtr[indexj] = mContactPatchPtr[j];
					mContactPatchPtr[j] = temp;
				}
			}
		}

		refinePatchesCoplanar();
		assignPatchesToManifolds(maxContactsPerManifold);

		mNumContacts = 0;
		mNumContactPatch = 0;

		if(isNotLastPatch)
		{
			for(PxU32 i = 0; i < PCM_MAX_CONTACTPATCH_SIZE; ++i)
				mContactPatchPtr[i] = &mContactPatch[i];
		}
	}
};
}

static bool pcmContactCustomGeometryGeometry(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxCustomGeometry& customGeom = checkedCast<PxCustomGeometry>(shape0);
	const PxGeometry& otherGeom = shape1;

	float breakingThreshold = 0.01f * params.mToleranceLength;
	bool usePCM = customGeom.callbacks->usePersistentContactManifold(customGeom, otherGeom, params.mToleranceLength, breakingThreshold);
	/*if (otherGeom.getType() == PxGeometryType::eCUSTOM)
	{
		float breakingThreshold1 = breakingThreshold;
		if (checkedCast<PxCustomGeometry>(shape1).callbacks->usePersistentContactManifold(otherGeom, breakingThreshold1))
		{
			breakingThreshold = PxMin(breakingThreshold, breakingThreshold1);
			usePCM = true;
		}
	}
	else if (otherGeom.getType() > PxGeometryType::eCONVEXMESH)
	{
		usePCM = true;
	}*/

	if(usePCM && cache.isMultiManifold())
	{
		MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

		const PxTransformV transf0 = loadTransformA(transform0);
		const PxTransformV transf1 = loadTransformA(transform1);
		const PxTransformV curRTrans = transf1.transformInv(transf0);

		const auto countContacts = [&multiManifold]() -> PxU32
		{
			PxU32 total = 0;
			for(PxU32 i = 0; i < multiManifold.mNumManifolds; ++i)
				total += multiManifold.getManifold(i)->mNumContacts;
			return total;
		};

		// BR: any contact pruned by refresh must force regen, or it stays gone until the transform gate trips.
		const PxMatTransformV aToB(curRTrans);
		const FloatV projectBreakingThreshold = FLoad(breakingThreshold * 0.8f);
		const PxU32 initialContacts = countContacts();
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, FLoad(params.mContactDistance));
		const bool bLostContacts = countContacts() != initialContacts;

		// BR: shape0's per-axis far extents are the rotation lever arms (manifold refreshes shape0-local points)
		PxVec3 farExtents(1.0f);
		{
			const PxBounds3 localBounds0 = customGeom.callbacks->getLocalBounds(customGeom);
			if(!localBounds0.isEmpty())
				farExtents = (localBounds0.getCenter().abs() + localBounds0.getExtents()).maximum(PxVec3(1.0f));
		}

		if(bLostContacts || multiManifold.invalidate(curRTrans, FLoad(breakingThreshold), FLoad(0.2f), V3LoadU(&farExtents.x)))
		{
			multiManifold.initialize();
			multiManifold.setRelativeTransform(curRTrans);

			const FloatV replaceBreakingThreshold = FLoad(breakingThreshold * 0.05f);

			ContactReceiverImpl receiver(multiManifold, transf0, transf1, replaceBreakingThreshold, params.mToleranceLength);

			customGeom.callbacks->generateContactsMultiManifold(
				customGeom, otherGeom, transform0, transform1,
				params.mContactDistance, params.mMeshContactMargin, params.mToleranceLength,
				receiver, renderOutput, cache.mPairData, contactBuffer.lowFidelity);

			receiver.processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, false);
		}

#if PCM_LOW_LEVEL_DEBUG
		multiManifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, transf1);
	}

	return customGeom.callbacks->generateContacts(customGeom, otherGeom, transform0, transform1,
		params.mContactDistance, params.mMeshContactMargin, params.mToleranceLength,
		cache, contactBuffer, renderOutput);
}

bool Gu::pcmContactGeometryCustomGeometry(GU_CONTACT_METHOD_ARGS)
{
	bool res = pcmContactCustomGeometryGeometry(shape1, shape0, transform1, transform0, params, cache, contactBuffer, renderOutput);

	for (PxU32 i = 0; i < contactBuffer.count; ++i)
		contactBuffer.contacts[i].normal = -contactBuffer.contacts[i].normal;

	return res;
}


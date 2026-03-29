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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geomutils/PxContactBuffer.h"
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

	ContactReceiverImpl(MultiplePersistentContactManifold& multiManifold,
		const PxTransformV& transf0, const PxTransformV& transf1,
		const FloatV& replaceBreakingThreshold)
		: mNumContacts(0)
		, mNumContactPatch(0)
		, mMultiManifold(multiManifold)
		, mTransf0(transf0)
		, mTransf1(transf1)
	{
		mSqReplaceBreakingThreshold = FMul(replaceBreakingThreshold, replaceBreakingThreshold);
		mAcceptanceEpsilon = FLoad(0.996f);

		for(PxU32 i = 0; i < PCM_MAX_CONTACTPATCH_SIZE; ++i)
			mContactPatchPtr[i] = &mContactPatch[i];
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
			mNumContacts = previousNumContacts + SinglePersistentContactManifold::reduceContacts(&mManifoldContacts[previousNumContacts], newContacts);
		}

		for(PxU32 i = previousNumContacts; i < mNumContacts; ++i)
		{
			for(PxU32 j = i + 1; j < mNumContacts; ++j)
			{
				const Vec3V dif = V3Sub(mManifoldContacts[j].mLocalPointB, mManifoldContacts[i].mLocalPointB);
				const FloatV d = V3Dot(dif, dif);
				if(FAllGrtr(mSqReplaceBreakingThreshold, d))
				{
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
			if(FAllGrtr(V3Dot(mContactPatch[mNumContactPatch - 1].mPatchNormal, localPatchNormal), mAcceptanceEpsilon))
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

		mMultiManifold.refineContactPatchConnective(mContactPatchPtr, mNumContactPatch, mManifoldContacts, mAcceptanceEpsilon);
		mMultiManifold.reduceManifoldContactsInDifferentPatches(mContactPatchPtr, mNumContactPatch, mManifoldContacts, mNumContacts, mSqReplaceBreakingThreshold);
		mMultiManifold.addManifoldContactPoints(mManifoldContacts, mNumContacts, mContactPatchPtr, mNumContactPatch, mSqReplaceBreakingThreshold, mAcceptanceEpsilon, maxContactsPerManifold);

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

		if(multiManifold.invalidate(curRTrans, FLoad(breakingThreshold)))
		{
			multiManifold.initialize();
			multiManifold.setRelativeTransform(curRTrans);

			const FloatV replaceBreakingThreshold = FLoad(breakingThreshold * 0.05f);

			ContactReceiverImpl receiver(multiManifold, transf0, transf1, replaceBreakingThreshold);

			customGeom.callbacks->generateContactsMultiManifold(
				customGeom, otherGeom, transform0, transform1,
				params.mContactDistance, params.mMeshContactMargin, params.mToleranceLength,
				receiver);

			receiver.processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, false);
		}
		else
		{
			const PxMatTransformV aToB(curRTrans);
			const FloatV projectBreakingThreshold = FLoad(breakingThreshold * 0.8f);
			multiManifold.refreshManifold(aToB, projectBreakingThreshold, FLoad(params.mContactDistance));
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


#ifndef DY_BUOYANCY_H
#define DY_BUOYANCY_H

#include "foundation/PxPlane.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxBounds3.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxCustomGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "PxsRigidBody.h"
#include "PxsWaterVolume.h"

namespace physx
{
namespace Dy
{

// Vertices and plane must share one space; the plane normal points out of the water.
class PolyhedronSubmergedVolume
{
public:
	struct Point
	{
		PxVec3	position;
		PxReal	distance;
		bool	above;
	};

	static const PxU32 MAX_POINTS = 256;

	PolyhedronSubmergedVolume(const PxVec3* points, PxU32 numPoints, const PxPlane& plane, Point* buffer) :
		mPoints			(buffer),
		mAllBelow		(true),
		mAllAbove		(true),
		mReferenceIdx	(0),
		mVolume			(0.0f),
		mCenter			(0.0f)
	{
		PxReal referenceDist = PX_MAX_F32;

		for(PxU32 i=0; i<numPoints; i++)
		{
			const PxVec3 p = points[i];
			const PxReal dist = plane.distance(p);
			const bool above = dist >= 0.0f;

			mAllAbove &= above;
			mAllBelow &= !above;

			if(referenceDist > dist)
			{
				mReferenceIdx = i;
				referenceDist = dist;
			}

			buffer[i].position = p;
			buffer[i].distance = dist;
			buffer[i].above = above;
		}
	}

	PX_FORCE_INLINE bool	allAbove()	const	{ return mAllAbove; }
	PX_FORCE_INLINE bool	allBelow()	const	{ return mAllBelow; }

	void addFace(PxU32 i1, PxU32 i2, PxU32 i3)
	{
		if(i1 == mReferenceIdx || i2 == mReferenceIdx || i3 == mReferenceIdx)
			return;

		const Point& ref = mPoints[mReferenceIdx];
		const Point& p1 = mPoints[i1];
		const Point& p2 = mPoints[i2];
		const Point& p3 = mPoints[i3];

		const PxU32 code = (p1.above ? 0 : 1u) | (p2.above ? 0 : 2u) | (p3.above ? 0 : 4u);

		PxReal volume;
		PxVec3 center;
		switch(code)
		{
		case 0:
			tetrahedron1(ref.position, ref.distance, p3.position, p3.distance, p2.position, p2.distance, p1.position, p1.distance, volume, center);
			break;
		case 1:
			tetrahedron2(ref.position, ref.distance, p1.position, p1.distance, p3.position, p3.distance, p2.position, p2.distance, volume, center);
			break;
		case 2:
			tetrahedron2(ref.position, ref.distance, p2.position, p2.distance, p1.position, p1.distance, p3.position, p3.distance, volume, center);
			break;
		case 4:
			tetrahedron2(ref.position, ref.distance, p3.position, p3.distance, p2.position, p2.distance, p1.position, p1.distance, volume, center);
			break;
		case 3:
			tetrahedron3(ref.position, ref.distance, p2.position, p2.distance, p1.position, p1.distance, p3.position, p3.distance, volume, center);
			break;
		case 5:
			tetrahedron3(ref.position, ref.distance, p1.position, p1.distance, p3.position, p3.distance, p2.position, p2.distance, volume, center);
			break;
		case 6:
			tetrahedron3(ref.position, ref.distance, p3.position, p3.distance, p2.position, p2.distance, p1.position, p1.distance, volume, center);
			break;
		case 7:
		default:
			tetrahedron4(ref.position, p3.position, p2.position, p1.position, volume, center);
			break;
		}

		mVolume += volume;
		mCenter += volume * center;
	}

	void getResult(PxReal& submergedVolume, PxVec3& centerOfBuoyancy) const
	{
		centerOfBuoyancy = mVolume > 0.0f ? mCenter / (4.0f * mVolume) : PxVec3(0.0f);
		submergedVolume = mVolume / 6.0f;
	}

private:
	// Volume*6 and centroid*4 of a fully submerged tetrahedron.
	static PX_FORCE_INLINE void tetrahedron4(const PxVec3& v1, const PxVec3& v2, const PxVec3& v3, const PxVec3& v4, PxReal& volumeTimes6, PxVec3& centerTimes4)
	{
		volumeTimes6 = PxMax((v1 - v4).dot((v2 - v4).cross(v3 - v4)), 0.0f);
		centerTimes4 = v1 + v2 + v3 + v4;
	}

	static PX_FORCE_INLINE PxVec3 planeIntersection(const PxVec3& v1, PxReal d1, const PxVec3& v2, PxReal d2)
	{
		const PxReal delta = d1 - d2;
		if(PxAbs(delta) < 1.0e-6f)
			return v1;
		return v1 + (v2 - v1) * (d1 / delta);
	}

	static void tetrahedron1(const PxVec3& v1, PxReal d1, const PxVec3& v2, PxReal d2, const PxVec3& v3, PxReal d3, const PxVec3& v4, PxReal d4, PxReal& volumeTimes6, PxVec3& centerTimes4)
	{
		const PxVec3 c2 = planeIntersection(v1, d1, v2, d2);
		const PxVec3 c3 = planeIntersection(v1, d1, v3, d3);
		const PxVec3 c4 = planeIntersection(v1, d1, v4, d4);

		tetrahedron4(v1, c2, c3, c4, volumeTimes6, centerTimes4);
	}

	static void tetrahedron2(const PxVec3& v1, PxReal d1, const PxVec3& v2, PxReal d2, const PxVec3& v3, PxReal d3, const PxVec3& v4, PxReal d4, PxReal& volumeTimes6, PxVec3& centerTimes4)
	{
		const PxVec3 c = planeIntersection(v1, d1, v3, d3);
		const PxVec3 d = planeIntersection(v1, d1, v4, d4);
		const PxVec3 e = planeIntersection(v2, d2, v4, d4);
		const PxVec3 f = planeIntersection(v2, d2, v3, d3);

		PxVec3 center1, center2, center3;
		PxReal volume1, volume2, volume3;
		tetrahedron4(e, f, v2, c, volume1, center1);
		tetrahedron4(e, v1, d, c, volume2, center2);
		tetrahedron4(e, v2, v1, c, volume3, center3);

		volumeTimes6 = volume1 + volume2 + volume3;
		centerTimes4 = volumeTimes6 > 0.0f ? (volume1 * center1 + volume2 * center2 + volume3 * center3) / volumeTimes6 : PxVec3(0.0f);
	}

	static void tetrahedron3(const PxVec3& v1, PxReal d1, const PxVec3& v2, PxReal d2, const PxVec3& v3, PxReal d3, const PxVec3& v4, PxReal d4, PxReal& volumeTimes6, PxVec3& centerTimes4)
	{
		const PxVec3 c1 = planeIntersection(v1, d1, v4, d4);
		const PxVec3 c2 = planeIntersection(v2, d2, v4, d4);
		const PxVec3 c3 = planeIntersection(v3, d3, v4, d4);

		PxVec3 dryCenter, totalCenter;
		PxReal dryVolume, totalVolume;
		tetrahedron4(c1, c2, c3, v4, dryVolume, dryCenter);
		tetrahedron4(v1, v2, v3, v4, totalVolume, totalCenter);

		volumeTimes6 = PxMax(totalVolume - dryVolume, 0.0f);
		centerTimes4 = volumeTimes6 > 0.0f ? (totalCenter * totalVolume - dryCenter * dryVolume) / volumeTimes6 : PxVec3(0.0f);
	}

	const Point*	mPoints;
	bool			mAllBelow;
	bool			mAllAbove;
	PxU32			mReferenceIdx;
	PxReal			mVolume;
	PxVec3			mCenter;
};

// Conservative plane-vs-AABB classification: 1 = fully above, -1 = fully below, 0 = intersecting.
PX_FORCE_INLINE PxI32 classifyPlaneBounds(const PxPlane& localPlane, const PxVec3& center, const PxVec3& extents)
{
	const PxReal reach = extents.x * PxAbs(localPlane.n.x) + extents.y * PxAbs(localPlane.n.y) + extents.z * PxAbs(localPlane.n.z);
	const PxReal centerDist = localPlane.distance(center);

	if(centerDist - reach >= 0.0f)
		return 1;

	if(centerDist + reach <= 0.0f)
		return -1;

	return 0;
}

PX_FORCE_INLINE void computeBoxSubmergedVolume(const PxBoxGeometry& box, const PxPlane& localPlane, PxReal& submerged, PxVec3& localCenter, PxReal& totalVolume)
{
	const PxVec3& e = box.halfExtents;
	totalVolume = 8.0f * e.x * e.y * e.z;
	submerged = 0.0f;
	localCenter = PxVec3(0.0f);

	const PxI32 classification = classifyPlaneBounds(localPlane, PxVec3(0.0f), e);

	if(classification > 0)
		return;

	if(classification < 0)
	{
		submerged = totalVolume;
		return;
	}

	const PxVec3 vertices[8] = {
		PxVec3(-e.x, -e.y, -e.z), PxVec3( e.x, -e.y, -e.z), PxVec3( e.x,  e.y, -e.z), PxVec3(-e.x,  e.y, -e.z),
		PxVec3(-e.x, -e.y,  e.z), PxVec3( e.x, -e.y,  e.z), PxVec3( e.x,  e.y,  e.z), PxVec3(-e.x,  e.y,  e.z)
	};

	PolyhedronSubmergedVolume::Point buffer[8];
	PolyhedronSubmergedVolume calculator(vertices, 8, localPlane, buffer);

	if(calculator.allAbove())
		return;

	if(calculator.allBelow())
	{
		submerged = totalVolume;
		return;
	}

	static const PxU32 indices[36] = {
		0, 1, 2,	0, 2, 3,
		4, 7, 6,	4, 6, 5,
		0, 4, 5,	0, 5, 1,
		3, 2, 6,	3, 6, 7,
		0, 3, 7,	0, 7, 4,
		1, 5, 6,	1, 6, 2
	};

	for(PxU32 i=0; i<36; i+=3)
		calculator.addFace(indices[i+2], indices[i+1], indices[i]);

	calculator.getResult(submerged, localCenter);
}

PX_FORCE_INLINE void computeSphereSubmergedVolume(const PxSphereGeometry& sphere, const PxPlane& localPlane, PxReal& submerged, PxVec3& localCenter, PxReal& totalVolume)
{
	const PxReal r = sphere.radius;
	totalVolume = (4.0f / 3.0f) * PxPi * r * r * r;
	submerged = 0.0f;
	localCenter = PxVec3(0.0f);

	const PxReal centerDist = localPlane.d;

	if(centerDist >= r)
		return;

	if(centerDist <= -r)
	{
		submerged = totalVolume;
		return;
	}

	const PxReal h = r - centerDist;
	submerged = PxPi * h * h * (3.0f * r - h) / 3.0f;

	const PxReal centroidOffset = 3.0f * (2.0f * r - h) * (2.0f * r - h) / (4.0f * (3.0f * r - h));
	localCenter = -localPlane.n * centroidOffset;
}

// Approximate: 32-slice midpoint integration along the capsule X axis.
PX_FORCE_INLINE void computeCapsuleSubmergedVolume(const PxCapsuleGeometry& capsule, const PxPlane& localPlane, PxReal& submerged, PxVec3& localCenter, PxReal& totalVolume)
{
	const PxReal r = capsule.radius;
	const PxReal hh = capsule.halfHeight;
	totalVolume = PxPi * r * r * (2.0f * hh) + (4.0f / 3.0f) * PxPi * r * r * r;
	submerged = 0.0f;
	localCenter = PxVec3(0.0f);

	const PxReal nyz = PxSqrt(localPlane.n.y * localPlane.n.y + localPlane.n.z * localPlane.n.z);
	const PxVec3 m = nyz > 1.0e-6f ? PxVec3(0.0f, localPlane.n.y / nyz, localPlane.n.z / nyz) : PxVec3(0.0f);

	const PxU32 numSlices = 32;
	const PxReal extent = hh + r;
	const PxReal dx = (2.0f * extent) / PxReal(numSlices);

	PxReal volume = 0.0f;
	PxVec3 firstMoment(0.0f);

	for(PxU32 i=0; i<numSlices; i++)
	{
		const PxReal x = -extent + (PxReal(i) + 0.5f) * dx;
		const PxReal axisOverhang = PxMax(PxAbs(x) - hh, 0.0f);
		const PxReal discRadiusSq = r * r - axisOverhang * axisOverhang;

		if(discRadiusSq <= 0.0f)
			continue;

		const PxReal discRadius = PxSqrt(discRadiusSq);
		const PxReal planeAtCenter = localPlane.n.x * x + localPlane.d;

		PxReal area;
		PxVec3 centroid(x, 0.0f, 0.0f);

		if(nyz <= 1.0e-6f)
		{
			if(planeAtCenter >= 0.0f)
				continue;

			area = PxPi * discRadiusSq;
		}
		else
		{
			const PxReal t = planeAtCenter / nyz;

			if(t >= discRadius)
				continue;

			if(t <= -discRadius)
			{
				area = PxPi * discRadiusSq;
			}
			else
			{
				const PxReal chord = PxSqrt(discRadiusSq - t * t);
				area = discRadiusSq * PxAcos(t / discRadius) - t * chord;

				if(area > 1.0e-6f)
					centroid -= m * ((2.0f / 3.0f) * chord * chord * chord / area);
			}
		}

		const PxReal sliceVolume = area * dx;
		volume += sliceVolume;
		firstMoment += centroid * sliceVolume;
	}

	if(volume > 0.0f)
	{
		submerged = PxMin(volume, totalVolume);
		localCenter = firstMoment / volume;
	}
}

PX_FORCE_INLINE void computeConvexSubmergedVolume(const PxConvexMeshGeometry& convex, const PxPlane& localPlane, PxReal& submerged, PxVec3& localCenter, PxReal& totalVolume)
{
	submerged = 0.0f;
	totalVolume = 0.0f;
	localCenter = PxVec3(0.0f);

	const PxConvexMesh* mesh = convex.convexMesh;
	if(!mesh)
		return;

	const PxU32 numVerts = mesh->getNbVertices();
	if(!numVerts || numVerts > PolyhedronSubmergedVolume::MAX_POINTS)
		return;

	PxReal meshVolume;
	PxMat33 inertia;
	PxVec3 centerOfMass;
	mesh->getMassInformation(meshVolume, inertia, centerOfMass);

	const bool identityScale = convex.scale.isIdentity();
	const PxMat33 scaleMat = identityScale ? PxMat33(PxIdentity) : convex.scale.toMat33();

	PxBounds3 localBounds = mesh->getLocalBounds();
	PxVec3 boundsCenter = localBounds.getCenter();
	PxVec3 boundsExtents = localBounds.getExtents();

	if(!identityScale)
	{
		const PxVec3& s = convex.scale.scale;
		meshVolume *= PxAbs(s.x * s.y * s.z);
		centerOfMass = scaleMat.transform(centerOfMass);

		boundsCenter = scaleMat.transform(boundsCenter);
		boundsExtents = PxVec3(
			PxAbs(scaleMat.column0.x) * localBounds.getExtents().x + PxAbs(scaleMat.column1.x) * localBounds.getExtents().y + PxAbs(scaleMat.column2.x) * localBounds.getExtents().z,
			PxAbs(scaleMat.column0.y) * localBounds.getExtents().x + PxAbs(scaleMat.column1.y) * localBounds.getExtents().y + PxAbs(scaleMat.column2.y) * localBounds.getExtents().z,
			PxAbs(scaleMat.column0.z) * localBounds.getExtents().x + PxAbs(scaleMat.column1.z) * localBounds.getExtents().y + PxAbs(scaleMat.column2.z) * localBounds.getExtents().z);
	}

	totalVolume = meshVolume;

	const PxI32 classification = classifyPlaneBounds(localPlane, boundsCenter, boundsExtents);

	if(classification > 0)
		return;

	if(classification < 0)
	{
		submerged = totalVolume;
		localCenter = centerOfMass;
		return;
	}

	const PxVec3* meshVerts = mesh->getVertices();
	PxVec3 scaledVerts[PolyhedronSubmergedVolume::MAX_POINTS];
	const PxVec3* verts = meshVerts;

	if(!identityScale)
	{
		for(PxU32 i=0; i<numVerts; i++)
			scaledVerts[i] = scaleMat.transform(meshVerts[i]);
		verts = scaledVerts;
	}

	PolyhedronSubmergedVolume::Point buffer[PolyhedronSubmergedVolume::MAX_POINTS];
	PolyhedronSubmergedVolume calculator(verts, numVerts, localPlane, buffer);

	if(calculator.allAbove())
		return;

	if(calculator.allBelow())
	{
		submerged = totalVolume;
		localCenter = centerOfMass;
		return;
	}

	const PxU8* indexBuffer = mesh->getIndexBuffer();
	const PxU32 numPolygons = mesh->getNbPolygons();

	for(PxU32 p=0; p<numPolygons; p++)
	{
		PxHullPolygon face;
		if(!mesh->getPolygonData(p, face))
			continue;

		for(PxU32 i=1; i+1<face.mNbVerts; i++)
		{
			calculator.addFace(
				indexBuffer[face.mIndexBase],
				indexBuffer[face.mIndexBase + i],
				indexBuffer[face.mIndexBase + i + 1]);
		}
	}

	calculator.getResult(submerged, localCenter);
}

PX_FORCE_INLINE void computeShapeSubmergedVolume(const PxGeometry& geom, const PxPlane& localPlane, PxReal& submerged, PxVec3& localCenter, PxReal& totalVolume)
{
	submerged = 0.0f;
	totalVolume = 0.0f;
	localCenter = PxVec3(0.0f);

	switch(geom.getType())
	{
	case PxGeometryType::eBOX:
		computeBoxSubmergedVolume(static_cast<const PxBoxGeometry&>(geom), localPlane, submerged, localCenter, totalVolume);
		break;
	case PxGeometryType::eSPHERE:
		computeSphereSubmergedVolume(static_cast<const PxSphereGeometry&>(geom), localPlane, submerged, localCenter, totalVolume);
		break;
	case PxGeometryType::eCAPSULE:
		computeCapsuleSubmergedVolume(static_cast<const PxCapsuleGeometry&>(geom), localPlane, submerged, localCenter, totalVolume);
		break;
	case PxGeometryType::eCONVEXMESH:
		computeConvexSubmergedVolume(static_cast<const PxConvexMeshGeometry&>(geom), localPlane, submerged, localCenter, totalVolume);
		break;
	case PxGeometryType::eCUSTOM:
		{
			const PxCustomGeometry& custom = static_cast<const PxCustomGeometry&>(geom);
			if(custom.callbacks)
			{
				if(!custom.callbacks->computeSubmergedVolume(geom, localPlane, submerged, localCenter, totalVolume))
				{
					submerged = 0.0f;
					totalVolume = 0.0f;
				}
			}
		}
		break;
	default:
		break;
	}
}

PX_FORCE_INLINE bool computeBodyBuoyancy(const PxsRigidBody& body, const PxsBodyCore& core, const PxsWaterVolume& waterVolume,
	PxVec3& buoyancyLinAccel, PxVec3& buoyancyAngAccel, PxReal& waterLinearDrag, PxReal& waterAngularDrag)
{
	const PxU32 numShapes = body.mNbBuoyancyShapes;
	if(!numShapes || core.inverseMass == 0.0f)
		return false;

	const PxsShapeCore* singleShape = reinterpret_cast<const PxsShapeCore*>(body.mBuoyancyShapes);
	const PxsShapeCore* const* shapeArray = reinterpret_cast<const PxsShapeCore* const*>(body.mBuoyancyShapes);

	const PxTransform actor2World = core.body2World * core.getBody2Actor().getInverse();
	const PxPlane worldPlane(PxVec3(0.0f, 0.0f, 1.0f), -waterVolume.surfaceHeight);

	PxReal totalSubmerged = 0.0f;
	PxReal totalVolume = 0.0f;
	PxVec3 firstMoment(0.0f);

	const bool useBounds = core.mFlags & PxRigidBodyFlag::eBUOYANCY_FROM_BOUNDS;
	PxBounds3 mergedBounds = PxBounds3::empty();

	for(PxU32 i=0; i<numShapes; i++)
	{
		const PxsShapeCore* shape = numShapes == 1 ? singleShape : shapeArray[i];
		const PxGeometry& geom = shape->mGeometry.getGeometry();

		if(useBounds)
		{
			PxBounds3 shapeBounds = PxBounds3::empty();

			switch(geom.getType())
			{
			case PxGeometryType::eBOX:
			{
				const PxVec3& e = static_cast<const PxBoxGeometry&>(geom).halfExtents;
				shapeBounds = PxBounds3(-e, e);
				break;
			}
			case PxGeometryType::eSPHERE:
			{
				const PxReal r = static_cast<const PxSphereGeometry&>(geom).radius;
				shapeBounds = PxBounds3(PxVec3(-r), PxVec3(r));
				break;
			}
			case PxGeometryType::eCAPSULE:
			{
				const PxCapsuleGeometry& capsule = static_cast<const PxCapsuleGeometry&>(geom);
				const PxVec3 e(capsule.halfHeight + capsule.radius, capsule.radius, capsule.radius);
				shapeBounds = PxBounds3(-e, e);
				break;
			}
			case PxGeometryType::eCONVEXMESH:
			{
				const PxConvexMeshGeometry& convex = static_cast<const PxConvexMeshGeometry&>(geom);
				if(convex.convexMesh)
				{
					const PxBounds3 localBounds = convex.convexMesh->getLocalBounds();
					shapeBounds = convex.scale.isIdentity() ? localBounds : PxBounds3::transformSafe(convex.scale.toMat33(), localBounds);
				}
				break;
			}
			default:
				break;
			}

			if(!shapeBounds.isEmpty())
			{
				mergedBounds.include(PxBounds3::transformSafe(shape->getTransform(), shapeBounds));
				continue;
			}
		}

		const PxTransform shape2World = actor2World * shape->getTransform();
		const PxPlane localPlane = worldPlane.inverseTransform(shape2World);

		PxReal submerged, shapeVolume;
		PxVec3 localCenter;
		computeShapeSubmergedVolume(geom, localPlane, submerged, localCenter, shapeVolume);

		totalVolume += shapeVolume;

		if(submerged > 0.0f)
		{
			totalSubmerged += submerged;
			firstMoment += shape2World.transform(localCenter) * submerged;
		}
	}

	if(!mergedBounds.isEmpty())
	{
		const PxTransform boxPose = actor2World * PxTransform(mergedBounds.getCenter());
		const PxPlane localPlane = worldPlane.inverseTransform(boxPose);
		const PxBoxGeometry boxGeom(mergedBounds.getExtents());

		PxReal submerged, boxVolume;
		PxVec3 localCenter;
		computeBoxSubmergedVolume(boxGeom, localPlane, submerged, localCenter, boxVolume);

		totalVolume += boxVolume;

		if(submerged > 0.0f)
		{
			totalSubmerged += submerged;
			firstMoment += boxPose.transform(localCenter) * submerged;
		}
	}

	if(totalSubmerged <= 1.0e-4f || totalVolume <= 0.0f)
		return false;

	const PxReal forceMagnitude = totalSubmerged * body.mBuoyancyScale * waterVolume.forceScale;
	const PxVec3 force(0.0f, 0.0f, forceMagnitude);

	const PxVec3 centerOfBuoyancy = firstMoment / totalSubmerged;
	const PxVec3 lever = centerOfBuoyancy - core.body2World.p;
	const PxVec3 worldTorque = lever.cross(force);

	buoyancyLinAccel = force * core.inverseMass;

	const PxVec3 localTorque = core.body2World.q.rotateInv(worldTorque);
	const PxVec3 localAngAccel = localTorque.multiply(core.inverseInertia);
	buoyancyAngAccel = core.body2World.q.rotate(localAngAccel);

	const PxReal ratio = PxClamp(totalSubmerged / totalVolume, 0.0f, 1.0f);
	waterLinearDrag = waterVolume.linearDrag * ratio;
	waterAngularDrag = waterVolume.angularDrag * ratio;
	return true;
}

}
}

#endif

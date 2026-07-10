// Water volume parameters shared between the simulation controller and the dynamics context.

#ifndef PXS_WATER_VOLUME_H
#define PXS_WATER_VOLUME_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{

// Matches the 0xffff encoding of PxsRigidBody::mWaterVolumeIndex.
#define PXS_INVALID_WATER_VOLUME 0xffffu

struct PxsWaterVolume
{
	PxReal	surfaceHeight;
	PxReal	forceScale;
	PxReal	linearDrag;
	PxReal	angularDrag;
};

}

#endif

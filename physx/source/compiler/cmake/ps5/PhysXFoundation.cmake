#
# Build PhysXFoundation - PS5
#

SET(PHYSXFOUNDATION_LIBTYPE STATIC)

SET(PHYSXFOUNDATION_PLATFORM_HEADERS
	${PHYSX_ROOT_DIR}/include/foundation/unix/PxUnixMathIntrinsics.h
	${PHYSX_ROOT_DIR}/include/foundation/unix/PxUnixIntrinsics.h
	${PHYSX_ROOT_DIR}/include/foundation/unix/PxUnixAoS.h
	${PHYSX_ROOT_DIR}/include/foundation/unix/PxUnixInlineAoS.h
	${PHYSX_ROOT_DIR}/include/foundation/unix/PxUnixTrigConstants.h
	${PHYSX_ROOT_DIR}/include/foundation/unix/PxUnixFPU.h
)
SOURCE_GROUP(include\\unix FILES ${PHYSXFOUNDATION_PLATFORM_HEADERS})

# No FdUnixSocket.cpp: PVD is disabled on PS5 and the BSD socket layer needs SceNet
SET(PHYSXFOUNDATION_PLATFORM_SOURCE
	${LL_SOURCE_DIR}/unix/FdUnixAtomic.cpp
	${LL_SOURCE_DIR}/unix/FdUnixMutex.cpp
	${LL_SOURCE_DIR}/unix/FdUnixSync.cpp
	${LL_SOURCE_DIR}/unix/FdUnixThread.cpp
	${LL_SOURCE_DIR}/unix/FdUnixPrintString.cpp
	${LL_SOURCE_DIR}/unix/FdUnixSList.cpp
	${LL_SOURCE_DIR}/unix/FdUnixTime.cpp
	${LL_SOURCE_DIR}/unix/FdUnixFPU.cpp
)
SOURCE_GROUP("src\\src\\unix" FILES ${PHYSXFOUNDATION_PLATFORM_SOURCE})

SET(PHYSXFOUNDATION_SSE2_FILES
	${PHYSX_ROOT_DIR}/include/foundation/unix/sse2/PxUnixSse2AoS.h
	${PHYSX_ROOT_DIR}/include/foundation/unix/sse2/PxUnixSse2InlineAoS.h
)

INSTALL(FILES ${PHYSXFOUNDATION_SSE2_FILES} DESTINATION include/foundation/unix/sse2)
INSTALL(FILES ${PHYSXFOUNDATION_PLATFORM_HEADERS} DESTINATION include/foundation/unix)

SET(PHYSXFOUNDATION_PLATFORM_FILES
	${PHYSXFOUNDATION_PLATFORM_SOURCE}
	${PHYSXFOUNDATION_PLATFORM_HEADERS}
	${PHYSXFOUNDATION_SSE2_FILES}
	${PHYSXFOUNDATION_RESOURCE_FILE}
)

SET(PHYSXFOUNDATION_PLATFORM_INCLUDES
)

# Use generator expressions to set config specific preprocessor definitions
SET(PHYSXFOUNDATION_COMPILE_DEFS
	# Common to all configurations
	${PHYSX_PS5_COMPILE_DEFS};${PXFOUNDATION_LIBTYPE_DEFS}

	$<$<CONFIG:debug>:${PHYSX_PS5_DEBUG_COMPILE_DEFS};>
	$<$<CONFIG:checked>:${PHYSX_PS5_CHECKED_COMPILE_DEFS};>
	$<$<CONFIG:profile>:${PHYSX_PS5_PROFILE_COMPILE_DEFS};>
	$<$<CONFIG:release>:${PHYSX_PS5_RELEASE_COMPILE_DEFS};>
)

SET(PXFOUNDATION_PLATFORM_LINK_FLAGS " ")

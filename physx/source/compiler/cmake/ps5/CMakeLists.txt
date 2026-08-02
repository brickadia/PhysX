## PS5 (Prospero) platform CMakeLists, modeled on the linux one.

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

SET(CLANG_WARNINGS "-ferror-limit=0 -Wall -Wextra \
    -Wno-unused-but-set-variable \
    -Wno-switch-default \
    -Wno-cast-qual \
    -Wno-invalid-offsetof \
    -Wno-unsafe-buffer-usage \
    -Wno-alloca \
    -Wno-atomic-implicit-seq-cst \
    -Wno-c++98-compat-pedantic \
    -Wno-c++98-compat \
    -Wno-cast-align \
    -Wno-conversion \
    -Wno-covered-switch-default \
    -Wno-deprecated \
    -Wno-documentation-deprecated-sync \
    -Wno-documentation-unknown-command \
    -Wno-exit-time-destructors \
    -Wno-extra-semi-stmt \
    -Wno-float-equal \
    -Wno-format-nonliteral \
    -Wno-global-constructors \
    -Wno-implicit-fallthrough \
    -Wno-inconsistent-missing-destructor-override \
    -Wno-inconsistent-missing-override \
    -Wno-missing-noreturn \
    -Wno-missing-prototypes \
    -Wno-missing-variable-declarations \
    -Wno-newline-eof \
    -Wno-non-virtual-dtor \
    -Wno-old-style-cast \
    -Wno-padded \
    -Wno-reserved-id-macro \
    -Wno-suggest-destructor-override \
    -Wno-suggest-override \
    -Wno-switch-enum \
    -Wno-undef \
    -Wno-undefined-reinterpret-cast \
    -Wno-unknown-warning-option \
    -Wno-unreachable-code \
    -Wno-unused-function \
    -Wno-unused-macros \
    -Wno-unused-member-function \
    -Wno-unused-private-field \
    -Wno-used-but-marked-unused \
    -Wno-weak-template-vtables \
    -Wno-weak-vtables \
    -Wno-zero-as-null-pointer-constant \
    -Wno-reserved-identifier \
    -Wno-undefined-func-template \
    -Wno-deprecated-copy-with-user-provided-copy \
    -Wno-missing-include-dirs \
    -Wno-unused-parameter \
    -Wno-expansion-to-defined \
    -Wno-bitwise-instead-of-logical \
    -Wno-return-type-c-linkage \
")

# SDK 13 libc++ requires C++20 or newer
SET(COMMON_CXX_FLAGS "-std=c++20 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fvisibility=hidden")

SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fstrict-aliasing -Wstrict-aliasing=2 -ffp-exception-behavior=maytrap -fno-fast-math -fno-unsafe-math-optimizations ${CLANG_WARNINGS}" CACHE INTERNAL "PhysX CXX")

SET(PHYSX_CXX_FLAGS_DEBUG   "-O0 -g" CACHE INTERNAL "PhysX Debug CXX Flags")
SET(PHYSX_CXX_FLAGS_CHECKED "-O3 -g" CACHE INTERNAL "PhysX Checked CXX Flags")
SET(PHYSX_CXX_FLAGS_PROFILE "-O3" CACHE INTERNAL "PhysX Profile CXX Flags")
IF(PX_THIN_LTO)
	SET(PHYSX_CXX_FLAGS_RELEASE "-O3 -flto=thin -fwhole-program-vtables" CACHE INTERNAL "PhysX Release CXX Flags")
ELSE()
	SET(PHYSX_CXX_FLAGS_RELEASE "-O3" CACHE INTERNAL "PhysX Release CXX Flags")
ENDIF()

SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(CMAKE_CXX_FLAGS_DEBUG   ${PHYSX_CXX_FLAGS_DEBUG})
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})

IF(PUBLIC_RELEASE)
	SET(PUBLIC_RELEASE_FLAG "PX_PUBLIC_RELEASE=1")
ELSE()
	SET(PUBLIC_RELEASE_FLAG "PX_PUBLIC_RELEASE=0")
ENDIF()

SET(NVTX_FLAG "PX_NVTX=0")

# No CUDA/GPU PhysX and no PVD tooling on PS5
SET(DISABLE_CUDA_DEF "DISABLE_CUDA_PHYSX")
SET(PVD_FLAGS "PX_SUPPORT_PVD=0;PX_SUPPORT_OMNI_PVD=0")

SET(PHYSX_PS5_COMPILE_DEFS "${PHYSX_AUTOBUILD};${PUBLIC_RELEASE_FLAG};${DISABLE_CUDA_DEF}" CACHE INTERNAL "Base PhysX preprocessor definitions")
SET(PHYSX_PS5_DEBUG_COMPILE_DEFS   "PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};${PVD_FLAGS}"  CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_PS5_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};${PVD_FLAGS}" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_PS5_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};${PVD_FLAGS}"  CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_PS5_RELEASE_COMPILE_DEFS "NDEBUG;${PVD_FLAGS}" CACHE INTERNAL "Release PhysX preprocessor definitions")

# Include all of the projects
INCLUDE(PhysXFoundation.cmake)
INCLUDE(LowLevel.cmake)
INCLUDE(LowLevelAABB.cmake)
INCLUDE(LowLevelDynamics.cmake)
INCLUDE(PhysX.cmake)
INCLUDE(PhysXCharacterKinematic.cmake)
INCLUDE(PhysXCommon.cmake)
INCLUDE(PhysXCooking.cmake)
INCLUDE(PhysXExtensions.cmake)
INCLUDE(PhysXVehicle.cmake)
INCLUDE(SceneQuery.cmake)
INCLUDE(SimulationController.cmake)
INCLUDE(PhysXPvdSDK.cmake)
INCLUDE(PhysXTask.cmake)

SET_PROPERTY(TARGET PhysX PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCharacterKinematic PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCommon PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCooking PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXExtensions PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXVehicle2 PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevel PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelAABB PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelDynamics PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SceneQuery PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SimulationController PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXPvdSDK PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXTask PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXFoundation PROPERTY FOLDER "PhysX SDK")

SET(PHYSXDISTRO_LIBS PhysXFoundation PhysX PhysXCharacterKinematic PhysXPvdSDK PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle2)

INSTALL(
	TARGETS ${PHYSXDISTRO_LIBS}
	EXPORT PhysXSDK
	DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>
	)

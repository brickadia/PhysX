FUNCTION(CompilerDumpVersion _OUTPUT_VERSION)

  EXEC_PROGRAM(${CMAKE_CXX_COMPILER}
    ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
    OUTPUT_VARIABLE COMPILER_VERSION
  )
  STRING(REGEX REPLACE "([0-9])\\.([0-9])(\\.[0-9])?" "\\1\\2"
    COMPILER_VERSION ${COMPILER_VERSION})

  SET(${_OUTPUT_VERSION} ${COMPILER_VERSION})
ENDFUNCTION()

FUNCTION(GetCompiler _ret)
	SET(COMPILER_SUFFIX "UNKNOWN")
	
	IF(CMAKE_CXX_COMPILER_ID STREQUAL "Intel"
		OR CMAKE_CXX_COMPILER MATCHES "icl"
		OR CMAKE_CXX_COMPILER MATCHES "icpc")
		IF(WIN32)
			SET (COMPILER_SUFFIX "iw")
		ELSE()
			SET (COMPILER_SUFFIX "il")
		ENDIF()
	ELSEIF (GHSMULTI)
		SET(COMPILER_SUFFIX "ghs")
	ELSEIF (MSVC_VERSION GREATER_EQUAL 1920)
		SET(COMPILER_SUFFIX "vc142")		
	ELSEIF (MSVC_VERSION GREATER_EQUAL 1910)
		SET(COMPILER_SUFFIX "vc141")
	ELSEIF (MSVC14)
		SET(COMPILER_SUFFIX "vc140")
	ELSEIF (MSVC12)
		SET(COMPILER_SUFFIX "vc120")
	ELSEIF (MSVC11)
		SET(COMPILER_SUFFIX "vc110")
	ELSEIF (MSVC10)
		SET(COMPILER_SUFFIX "vc100")
	ELSEIF (MSVC90)
		SET(COMPILER_SUFFIX "vc90")
	ELSEIF (MSVC80)
		SET(COMPILER_SUFFIX "vc80")
	ELSEIF (MSVC71)
		SET(COMPILER_SUFFIX "vc71")
	ELSEIF (MSVC70) # Good luck!
		SET(COMPILER_SUFFIX "vc7") # yes, this is correct
	ELSEIF (MSVC60) # Good luck!
		SET(COMPILER_SUFFIX "vc6") # yes, this is correct
	ELSEIF (BORLAND)
		SET(COMPILER_SUFFIX "bcb")
	ELSEIF(CMAKE_CXX_COMPILER_ID STREQUAL "SunPro")
		SET(COMPILER_SUFFIX "sw")
	ELSEIF(CMAKE_CXX_COMPILER_ID STREQUAL "XL")
		SET(COMPILER_SUFFIX "xlc")
	ELSEIF (MINGW)
		CompilerDumpVersion(_COMPILER_VERSION)
		SET(COMPILER_SUFFIX "mgw${_COMPILER_VERSION}")
	ELSEIF (UNIX)
		IF (CMAKE_COMPILER_IS_GNUCXX)
			CompilerDumpVersion(_COMPILER_VERSION)
			IF(APPLE)
		      		# on Mac OS X/Darwin is "xgcc".
		      		SET(COMPILER_SUFFIX "xgcc${_COMPILER_VERSION}")
			ELSE()
		  		SET(COMPILER_SUFFIX "gcc${_COMPILER_VERSION}")
			ENDIF()
		ENDIF()
	ELSE()
		# add clang!
		SET(COMPILER_SUFFIX "")
	ENDIF()
	
	SET(${_ret} ${COMPILER_SUFFIX} PARENT_SCOPE)	
ENDFUNCTION()

FUNCTION(GetStaticCRTString _ret)
	IF(NOT TARGET_BUILD_PLATFORM STREQUAL "windows")
		return()
	ENDIF()

	IF (NV_USE_STATIC_WINCRT)
		SET(CRT_STRING "mt")
	ELSE()
		SET(CRT_STRING "md")
	ENDIF()

	SET(${_ret} ${CRT_STRING} PARENT_SCOPE)
ENDFUNCTION()

FUNCTION (GetPlatformBinName PLATFORM_BIN_NAME LIBPATH_SUFFIX)
	SET(RETVAL "UNKNOWN")

	GetCompiler(COMPILER)
	
	IF(TARGET_BUILD_PLATFORM STREQUAL "windows")
		GetStaticCRTString(CRT_STRING)
		SET(RETVAL "win.x86_${LIBPATH_SUFFIX}.${COMPILER}.${CRT_STRING}")
	ELSEIF(TARGET_BUILD_PLATFORM STREQUAL "mac")
		SET(RETVAL "mac.x86_${LIBPATH_SUFFIX}")
	ELSEIF(TARGET_BUILD_PLATFORM STREQUAL "switch")
		IF (${CMAKE_GENERATOR_PLATFORM} STREQUAL "NX32")
			SET(RETVAL "switch32")
		ELSEIF (${CMAKE_GENERATOR_PLATFORM} STREQUAL "NX64")
		SET(RETVAL "switch64")
		ENDIF()
	ELSEIF(TARGET_BUILD_PLATFORM STREQUAL "linux")
		IF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
			SET(RETVAL "linux.clang")
		ELSEIF(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
			SET(RETVAL "linux.aarch64")
		ENDIF()
	ENDIF()

	SET(${PLATFORM_BIN_NAME} ${RETVAL} PARENT_SCOPE)
ENDFUNCTION()

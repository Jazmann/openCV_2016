message (STATUS "Setting up iPhoneOS toolchain")
set (IPHONEOS TRUE)

# Standard settings
set (CMAKE_SYSTEM_NAME iOS)
# Include extra modules for the iOS platform files
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/platforms/ios/cmake/Modules")

# Force the compilers to clang for iOS
include (CMakeForceCompiler)
#CMAKE_FORCE_C_COMPILER (clang GNU)
#CMAKE_FORCE_CXX_COMPILER (clang++ GNU)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")

#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#CHECK_CXX_COMPILER_FLAG("-std=c++98" COMPILER_SUPPORTS_CXX98)
#if(COMPILER_SUPPORTS_CXX11)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
#elseif(COMPILER_SUPPORTS_CXX0X)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -stdlib=libc++")
#elseif(COMPILER_SUPPORTS_CXX98)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++98 -stdlib=libstdc++")
#else()
#    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 or C++98 support. Please use a different C++ compiler.")
#endif()

set (CMAKE_C_SIZEOF_DATA_PTR 4)
set (CMAKE_C_HAS_ISYSROOT 1)
set (CMAKE_C_COMPILER_ABI ELF)
set (CMAKE_CXX_SIZEOF_DATA_PTR 4)
set (CMAKE_CXX_HAS_ISYSROOT 1)
set (CMAKE_CXX_COMPILER_ABI ELF)

# Skip the platform compiler checks for cross compiling
set (CMAKE_CXX_COMPILER_WORKS TRUE)
set (CMAKE_C_COMPILER_WORKS TRUE)

# Search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
#   for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

message (STATUS "iPhoneOS toolchain loaded")

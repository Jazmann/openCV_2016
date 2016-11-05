/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Copyright (C) 2015, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef OPENCV_CORE_CVDEF_H
#define OPENCV_CORE_CVDEF_H

//! @addtogroup core_utils
//! @{

/****************************************************************************************\
 *                                    C++ Move semantics                                  *
 \****************************************************************************************/

#ifndef CV_CXX_MOVE_SEMANTICS
#  if __cplusplus >= 201103L || defined(__GXX_EXPERIMENTAL_CXX0X__) || defined(_MSC_VER) && _MSC_VER >= 1600
#    define CV_CXX_MOVE_SEMANTICS 1
#  elif defined(__clang)
#    if __has_feature(cxx_rvalue_references)
#      define CV_CXX_MOVE_SEMANTICS 1
#    endif
#  endif
#else
#  if CV_CXX_MOVE_SEMANTICS == 0
#    undef CV_CXX_MOVE_SEMANTICS
#  endif
#endif

#ifdef CV_CXX_MOVE_SEMANTICS
// Compiling in at least C++11 mode.
#  define CV_LANG_CXX11 1
#  define _STD
#  define REGISTER
# else
#  define CV_LANG_CXX11 0
#  define _STD std::
#  define REGISTER register
# endif


#if !defined _CRT_SECURE_NO_DEPRECATE && defined _MSC_VER && _MSC_VER > 1300
#  define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio warnings */
#endif

// undef problematic defines sometimes defined by system headers (windows.h in particular)
#undef small
#undef min
#undef max
#undef abs
#undef Complex

#if !defined _CRT_SECURE_NO_DEPRECATE && defined _MSC_VER && _MSC_VER > 1300
#  define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio warnings */
#endif

#include <limits.h>
#include "opencv2/core/hal/interface.h"

#if defined __ICL
#  define CV_ICC   __ICL
#elif defined __ICC
#  define CV_ICC   __ICC
#elif defined __ECL
#  define CV_ICC   __ECL
#elif defined __ECC
#  define CV_ICC   __ECC
#elif defined __INTEL_COMPILER
#  define CV_ICC   __INTEL_COMPILER
#endif

#ifndef CV_INLINE
#  if defined __cplusplus
#    define CV_INLINE static inline
#  elif defined _MSC_VER
#    define CV_INLINE __inline
#  else
#    define CV_INLINE static
#  endif
#endif

#if defined CV_ICC && !defined CV_ENABLE_UNROLLED
#  define CV_ENABLE_UNROLLED 0
#else
#  define CV_ENABLE_UNROLLED 1
#endif

#ifdef __GNUC__
#  define CV_DECL_ALIGNED(x) __attribute__ ((aligned (x)))
#elif defined _MSC_VER
#  define CV_DECL_ALIGNED(x) __declspec(align(x))
#else
#  define CV_DECL_ALIGNED(x)
#endif

/* CPU features and intrinsics support */
#define CV_CPU_NONE             0
#define CV_CPU_MMX              1
#define CV_CPU_SSE              2
#define CV_CPU_SSE2             3
#define CV_CPU_SSE3             4
#define CV_CPU_SSSE3            5
#define CV_CPU_SSE4_1           6
#define CV_CPU_SSE4_2           7
#define CV_CPU_POPCNT           8
#define CV_CPU_FP16             9
#define CV_CPU_AVX              10
#define CV_CPU_AVX2             11
#define CV_CPU_FMA3             12

#define CV_CPU_AVX_512F         13
#define CV_CPU_AVX_512BW        14
#define CV_CPU_AVX_512CD        15
#define CV_CPU_AVX_512DQ        16
#define CV_CPU_AVX_512ER        17
#define CV_CPU_AVX_512IFMA512   18
#define CV_CPU_AVX_512PF        19
#define CV_CPU_AVX_512VBMI      20
#define CV_CPU_AVX_512VL        21

#define CV_CPU_NEON   100

// when adding to this list remember to update the following enum
#define CV_HARDWARE_MAX_FEATURE 255

/** @brief Available CPU features.
*/
enum CpuFeatures {
    CPU_MMX             = 1,
    CPU_SSE             = 2,
    CPU_SSE2            = 3,
    CPU_SSE3            = 4,
    CPU_SSSE3           = 5,
    CPU_SSE4_1          = 6,
    CPU_SSE4_2          = 7,
    CPU_POPCNT          = 8,
    CPU_FP16            = 9,
    CPU_AVX             = 10,
    CPU_AVX2            = 11,
    CPU_FMA3            = 12,

    CPU_AVX_512F        = 13,
    CPU_AVX_512BW       = 14,
    CPU_AVX_512CD       = 15,
    CPU_AVX_512DQ       = 16,
    CPU_AVX_512ER       = 17,
    CPU_AVX_512IFMA512  = 18,
    CPU_AVX_512PF       = 19,
    CPU_AVX_512VBMI     = 20,
    CPU_AVX_512VL       = 21,

    CPU_NEON            = 100
};

// do not include SSE/AVX/NEON headers for NVCC compiler
#ifndef __CUDACC__

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#  include <emmintrin.h>
#  define CV_MMX 1
#  define CV_SSE 1
#  define CV_SSE2 1
#  if defined __SSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#    include <pmmintrin.h>
#    define CV_SSE3 1
#  endif
#  if defined __SSSE3__  || (defined _MSC_VER && _MSC_VER >= 1500)
#    include <tmmintrin.h>
#    define CV_SSSE3 1
#  endif
#  if defined __SSE4_1__ || (defined _MSC_VER && _MSC_VER >= 1500)
#    include <smmintrin.h>
#    define CV_SSE4_1 1
#  endif
#  if defined __SSE4_2__ || (defined _MSC_VER && _MSC_VER >= 1500)
#    include <nmmintrin.h>
#    define CV_SSE4_2 1
#  endif
#  if defined __POPCNT__ || (defined _MSC_VER && _MSC_VER >= 1500)
#    ifdef _MSC_VER
#      include <nmmintrin.h>
#    else
#      include <popcntintrin.h>
#    endif
#    define CV_POPCNT 1
#  endif
#  if defined __AVX__ || (defined _MSC_VER && _MSC_VER >= 1600 && 0)
// MS Visual Studio 2010 (2012?) has no macro pre-defined to identify the use of /arch:AVX
// See: http://connect.microsoft.com/VisualStudio/feedback/details/605858/arch-avx-should-define-a-predefined-macro-in-x64-and-set-a-unique-value-for-m-ix86-fp-in-win32
#    include <immintrin.h>
#    define CV_AVX 1
#    if defined(_XCR_XFEATURE_ENABLED_MASK)
#      define __xgetbv() _xgetbv(_XCR_XFEATURE_ENABLED_MASK)
#    else
#      define __xgetbv() 0
#    endif
#  endif
#  if defined __AVX2__ || (defined _MSC_VER && _MSC_VER >= 1800 && 0)
#    include <immintrin.h>
#    define CV_AVX2 1
#    if defined __FMA__
#      define CV_FMA3 1
#    endif
#  endif
#endif

#if (defined WIN32 || defined _WIN32) && defined(_M_ARM)
# include <Intrin.h>
# include <arm_neon.h>
# define CV_NEON 1
# define CPU_HAS_NEON_FEATURE (true)
#elif defined(__ARM_NEON__) || (defined (__ARM_NEON) && defined(__aarch64__))
#  include <arm_neon.h>
#  define CV_NEON 1
#endif

#if defined __GNUC__ && defined __arm__ && (defined __ARM_PCS_VFP || defined __ARM_VFPV3__ || defined __ARM_NEON__) && !defined __SOFTFP__
#  define CV_VFP 1
#endif

#endif // __CUDACC__

#ifndef CV_POPCNT
#define CV_POPCNT 0
#endif
#ifndef CV_MMX
#  define CV_MMX 0
#endif
#ifndef CV_SSE
#  define CV_SSE 0
#endif
#ifndef CV_SSE2
#  define CV_SSE2 0
#endif
#ifndef CV_SSE3
#  define CV_SSE3 0
#endif
#ifndef CV_SSSE3
#  define CV_SSSE3 0
#endif
#ifndef CV_SSE4_1
#  define CV_SSE4_1 0
#endif
#ifndef CV_SSE4_2
#  define CV_SSE4_2 0
#endif
#ifndef CV_AVX
#  define CV_AVX 0
#endif
#ifndef CV_AVX2
#  define CV_AVX2 0
#endif
#ifndef CV_FMA3
#  define CV_FMA3 0
#endif
#ifndef CV_AVX_512F
#  define CV_AVX_512F 0
#endif
#ifndef CV_AVX_512BW
#  define CV_AVX_512BW 0
#endif
#ifndef CV_AVX_512CD
#  define CV_AVX_512CD 0
#endif
#ifndef CV_AVX_512DQ
#  define CV_AVX_512DQ 0
#endif
#ifndef CV_AVX_512ER
#  define CV_AVX_512ER 0
#endif
#ifndef CV_AVX_512IFMA512
#  define CV_AVX_512IFMA512 0
#endif
#ifndef CV_AVX_512PF
#  define CV_AVX_512PF 0
#endif
#ifndef CV_AVX_512VBMI
#  define CV_AVX_512VBMI 0
#endif
#ifndef CV_AVX_512VL
#  define CV_AVX_512VL 0
#endif

#ifndef CV_NEON
#  define CV_NEON 0
#endif

#ifndef CV_VFP
#  define CV_VFP 0
#endif

/* fundamental constants */
#define CV_PI   3.1415926535897932384626433832795
#define CV_2PI 6.283185307179586476925286766559
#define CV_LOG2 0.69314718055994530941723212145818

#if defined __ARM_FP16_FORMAT_IEEE \
    && !defined __CUDACC__
#  define CV_FP16_TYPE 1
#else
#  define CV_FP16_TYPE 0
#endif

typedef union Cv16suf
{
    short i;
#if CV_FP16_TYPE
    __fp16 h;
#endif
    struct _fp16Format
    {
        unsigned int significand : 10;
        unsigned int exponent    : 5;
        unsigned int sign        : 1;
    } fmt;
}
Cv16suf;

typedef union Cv32suf
{
    int i;
    unsigned u;
    float f;
    struct _fp32Format
    {
        unsigned int significand : 23;
        unsigned int exponent    : 8;
        unsigned int sign        : 1;
    } fmt;
}
Cv32suf;

typedef union Cv64suf
{
    int64 i;
    uint64 u;
    double f;
}
Cv64suf;

#define OPENCV_ABI_COMPATIBILITY 300

#ifdef __OPENCV_BUILD
#  define DISABLE_OPENCV_24_COMPATIBILITY
#endif

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__) && defined CVAPI_EXPORTS
#  define CV_EXPORTS __declspec(dllexport)
#elif defined __GNUC__ && __GNUC__ >= 4
#  define CV_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define CV_EXPORTS
#endif

#ifndef CV_EXTERN_C
#  ifdef __cplusplus
#    define CV_EXTERN_C extern "C"
#  else
#    define CV_EXTERN_C
#  endif
#endif

/* special informative macros for wrapper generators */
#define CV_EXPORTS_W CV_EXPORTS
#define CV_EXPORTS_W_SIMPLE CV_EXPORTS
#define CV_EXPORTS_AS(synonym) CV_EXPORTS
#define CV_EXPORTS_W_MAP CV_EXPORTS
#define CV_IN_OUT
#define CV_OUT
#define CV_PROP
#define CV_PROP_RW
#define CV_WRAP
#define CV_WRAP_AS(synonym)
//
///****************************************************************************************\
//*                                  Matrix type (Mat)                                     *
//\****************************************************************************************/
//// CV_CN_MAX reserves 9 bits for the channel numbers.
//#define CV_CN_MAX     (1 << 9)
//// CV_CN_SHIFT resrves the first 3 bits for the Data Types (CV_8U, etc)
//#define CV_CN_SHIFT   3
//// CV_DEPTH_MAX is the maximum number of Data types allowed.
//#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)
//
//// TYPE_TAB_ORDER puts a list of elements which are ordered by type into a list ordered by type numbering.
//// The C++11 types are (uint8, int8, uint16, int16, uint32, int32, uint64, int64, float, double)
//// The current type tab order which does not use all C++11 types is
//// {CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F, CV_64F, CV_USRTYPE1}
////
//// If CV_CN_SHIFT is increased to 4 bits then 16 types can be defined.
//// This allows all 10 types to be included with 6 undefined types.
//// (uint8, int8, uint16, int16, uint32, int32, uint64, int64, float, double, usr1, usr2, usr3, usr4, usr5, usr6)
//// TYPE_TAB_ORDER would then be
//// #define TYPE_TAB_ORDER(cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4) {cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4}
////
//// For now the original order is preserved
//
//#define TYPE_TAB_ORDER(cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4, cvUSRTYPE5, cvUSRTYPE6) {cv8U, cv8S, cv16U, cv16S, cv32S, cv32F, cv64F, cvUSRTYPE1}
//#define D_TYPE_TAB_ORDER(cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4, cvUSRTYPE5, cvUSRTYPE6) {cv8U, cv8S, cv16U, cv16S, cv32S, cv32F, cv64F, cvUSRTYPE1}
//
//#define CV_8U_EXISTS 1
//#define CV_8S_EXISTS 1
//#define CV_16U_EXISTS 1
//#define CV_16S_EXISTS 1
//#define CV_32U_EXISTS 0
//#define CV_32S_EXISTS 1
//#define CV_64U_EXISTS 0
//#define CV_64S_EXISTS 0
//#define CV_32F_EXISTS 1
//#define CV_64F_EXISTS 1
//#define CV_USRTYPE1_EXISTS 0
//#define CV_USRTYPE2_EXISTS 0
//#define CV_USRTYPE3_EXISTS 0
//#define CV_USRTYPE4_EXISTS 0
//#define CV_USRTYPE5_EXISTS 0
//#define CV_USRTYPE6_EXISTS 0
//
//#define IF_CV_8U_EXISTS(body) body
//#define IF_CV_8S_EXISTS(body) body
//#define IF_CV_16U_EXISTS(body) body
//#define IF_CV_16S_EXISTS(body) body
//#define IF_CV_32U_EXISTS(body)
//#define IF_CV_32S_EXISTS(body) body
//#define IF_CV_64U_EXISTS(body)
//#define IF_CV_64S_EXISTS(body)
//#define IF_CV_32F_EXISTS(body) body
//#define IF_CV_64F_EXISTS(body) body
//#define IF_CV_USRTYPE1_EXISTS(body)
//#define IF_CV_USRTYPE2_EXISTS(body)
//#define IF_CV_USRTYPE3_EXISTS(body)
//#define IF_CV_USRTYPE4_EXISTS(body)
//#define IF_CV_USRTYPE5_EXISTS(body)
//#define IF_CV_USRTYPE6_EXISTS(body)
//
//#define CV_8U   0
//#define CV_8U_DEPTH_BITS_LOG2  3
//#define CV_8U_DEPTH_BYTES_LOG2 0
//#define CV_8U_TYPE             _STD uint8_t
//#define CV_8U_MAX              UINT8_MAX
//#define CV_8U_MIN              0
//
//#define CV_8S   1
//#define CV_8S_DEPTH_BITS_LOG2  3
//#define CV_8S_DEPTH_BYTES_LOG2 0
//#define CV_8S_TYPE             _STD int8_t
//#define CV_8S_MAX              INT8_MAX
//#define CV_8S_MIN              INT8_MIN
//
//#define CV_16U  2
//#define CV_16U_DEPTH_BITS_LOG2  4
//#define CV_16U_DEPTH_BYTES_LOG2 1
//#define CV_16U_TYPE             _STD uint16_t
//#define CV_16U_MAX              UINT16_MAX
//#define CV_16U_MIN              0
//
//#define CV_16S  3
//#define CV_16S_DEPTH_BITS_LOG2  4
//#define CV_16S_DEPTH_BYTES_LOG2 1
//#define CV_16S_TYPE             _STD int16_t
//#define CV_16S_MAX              INT16_MAX
//#define CV_16S_MIN              INT16_MIN
//
//#define CV_32U  8
//#define CV_32U_DEPTH_BITS_LOG2  5
//#define CV_32U_DEPTH_BYTES_LOG2 2
//#define CV_32U_TYPE             _STD uint32_t
//#define CV_32U_MAX              UINT32_MAX
//#define CV_32U_MIN              0
//
//#define CV_32S  4
//#define CV_32S_DEPTH_BITS_LOG2  5
//#define CV_32S_DEPTH_BYTES_LOG2 2
//#define CV_32S_TYPE             _STD int32_t
//#define CV_32S_MAX              INT32_MAX
//#define CV_32S_MIN              INT32_MIN
//
//#define CV_64U  9
//#define CV_64U_DEPTH_BITS_LOG2  6
//#define CV_64U_DEPTH_BYTES_LOG2 3
//#define CV_64U_TYPE             _STD uint64_t
//#define CV_64U_MAX              UINT64_MAX
//#define CV_64U_MIN              0
//
//#define CV_64S  10
//#define CV_64S_DEPTH_BITS_LOG2  6
//#define CV_64S_DEPTH_BYTES_LOG2 3
//#define CV_64S_TYPE             _STD int64_t
//#define CV_64S_MAX              INT64_MAX
//#define CV_64S_MIN              INT64_MIN
//
//#define CV_32F  5
//#define CV_32F_DEPTH_BITS_LOG2  5
//#define CV_32F_DEPTH_BYTES_LOG2 2
//#define CV_32F_TYPE             float
//#define CV_32F_MAX              FLT_MAX
//#define CV_32F_MIN              FLT_MIN
//
//#define CV_64F  6
//#define CV_64F_DEPTH_BITS_LOG2  6
//#define CV_64F_DEPTH_BYTES_LOG2 3
//#define CV_64F_TYPE             double
//#define CV_64F_MAX              DBL_MAX
//#define CV_64F_MIN              DBL_MIN
//
//#define CV_USRTYPE1 7
//#define CV_USRTYPE2 11
//#define CV_USRTYPE3 12
//#define CV_USRTYPE4 13
//#define CV_USRTYPE5 14
//#define CV_USRTYPE6 15
//
//#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
//#define CV_MAT_DEPTH(type)    ((type) & (CV_MAT_DEPTH_MASK)) // CV_MAT_DEPTH(flags) applies the mask to the bits in flags.
//
//#define CV_DEPTH_BYTES_MAGIC ( \
//(CV_64F_DEPTH_BYTES_LOG2 << (CV_64F *2))|(CV_32F_DEPTH_BYTES_LOG2 << (CV_32F *2))|\
//(CV_64S_DEPTH_BYTES_LOG2 << (CV_64S *2))|(CV_64U_DEPTH_BYTES_LOG2 << (CV_64U *2))|\
//(CV_32S_DEPTH_BYTES_LOG2 << (CV_32S *2))|(CV_32U_DEPTH_BYTES_LOG2 << (CV_32U *2))|\
//(CV_16S_DEPTH_BYTES_LOG2 << (CV_16S *2))|(CV_16U_DEPTH_BYTES_LOG2 << (CV_16U *2))|\
//(CV_8S_DEPTH_BYTES_LOG2  << (CV_8S  *2))|(CV_8U_DEPTH_BYTES_LOG2  << (CV_8U  *2)) )
//
//#define CV_MAT_DEPTH_BYTES(type) (1 << ( (CV_DEPTH_BYTES_MAGIC >> (CV_MAT_DEPTH(type)*2)) &3) )
//#define CV_DEPTH_BYTES(type)     CV_MAT_DEPTH_BYTES(type) // Depreciated
//
//#define CV_DEPTH_BITS_MAGIC ( \
//(uint64_t(CV_64F_DEPTH_BITS_LOG2) << (CV_64F *3))|(uint64_t(CV_32F_DEPTH_BITS_LOG2) << (CV_32F *3))|\
//(uint64_t(CV_64S_DEPTH_BITS_LOG2) << (CV_64S *3))|(uint64_t(CV_64U_DEPTH_BITS_LOG2) << (CV_64U *3))|\
//(uint64_t(CV_32S_DEPTH_BITS_LOG2) << (CV_32S *3))|(uint64_t(CV_32U_DEPTH_BITS_LOG2) << (CV_32U *3))|\
//(uint64_t(CV_16S_DEPTH_BITS_LOG2) << (CV_16S *3))|(uint64_t(CV_16U_DEPTH_BITS_LOG2) << (CV_16U *3))|\
//(uint64_t(CV_8S_DEPTH_BITS_LOG2)  << (CV_8S  *3))|(uint64_t(CV_8U_DEPTH_BITS_LOG2)  << (CV_8U  *3)) )
//
//#define CV_MAT_DEPTH_BITS(type)   (1 << ( (CV_DEPTH_BITS_MAGIC >> ((CV_MAT_DEPTH(type))*3)) & 7) )
//#define CV_DEPTH_BITS(type)       CV_MAT_DEPTH_BITS(type) // Depreciated
//
//// CV_MAKETYPE(depth,cn) generated an integer using the right-most CV_CN_SHIFT bits for the depth and the rest for the channels.
//
//#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
//#define CV_MAKE_TYPE CV_MAKETYPE
//
//#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
//#define CV_8UC2 CV_MAKETYPE(CV_8U,2)
//#define CV_8UC3 CV_MAKETYPE(CV_8U,3)
//#define CV_8UC4 CV_MAKETYPE(CV_8U,4)
//#define CV_8UC(n) CV_MAKETYPE(CV_8U,(n))
//
//#define CV_8SC1 CV_MAKETYPE(CV_8S,1)
//#define CV_8SC2 CV_MAKETYPE(CV_8S,2)
//#define CV_8SC3 CV_MAKETYPE(CV_8S,3)
//#define CV_8SC4 CV_MAKETYPE(CV_8S,4)
//#define CV_8SC(n) CV_MAKETYPE(CV_8S,(n))
//
//#define CV_16UC1 CV_MAKETYPE(CV_16U,1)
//#define CV_16UC2 CV_MAKETYPE(CV_16U,2)
//#define CV_16UC3 CV_MAKETYPE(CV_16U,3)
//#define CV_16UC4 CV_MAKETYPE(CV_16U,4)
//#define CV_16UC(n) CV_MAKETYPE(CV_16U,(n))
//
//#define CV_16SC1 CV_MAKETYPE(CV_16S,1)
//#define CV_16SC2 CV_MAKETYPE(CV_16S,2)
//#define CV_16SC3 CV_MAKETYPE(CV_16S,3)
//#define CV_16SC4 CV_MAKETYPE(CV_16S,4)
//#define CV_16SC(n) CV_MAKETYPE(CV_16S,(n))
//
//#define CV_32SC1 CV_MAKETYPE(CV_32S,1)
//#define CV_32SC2 CV_MAKETYPE(CV_32S,2)
//#define CV_32SC3 CV_MAKETYPE(CV_32S,3)
//#define CV_32SC4 CV_MAKETYPE(CV_32S,4)
//#define CV_32SC(n) CV_MAKETYPE(CV_32S,(n))
//
//#define CV_32FC1 CV_MAKETYPE(CV_32F,1)
//#define CV_32FC2 CV_MAKETYPE(CV_32F,2)
//#define CV_32FC3 CV_MAKETYPE(CV_32F,3)
//#define CV_32FC4 CV_MAKETYPE(CV_32F,4)
//#define CV_32FC(n) CV_MAKETYPE(CV_32F,(n))
//
//#define CV_64FC1 CV_MAKETYPE(CV_64F,1)
//#define CV_64FC2 CV_MAKETYPE(CV_64F,2)
//#define CV_64FC3 CV_MAKETYPE(CV_64F,3)
//#define CV_64FC4 CV_MAKETYPE(CV_64F,4)
//#define CV_64FC(n) CV_MAKETYPE(CV_64F,(n))
//
//// To get back the information put into CV_MAKETYPE( depth_Type, cn) use
//// int depth_Type = CV_MAT_DEPTH(CV_##tC#)
//// int cn = CV_MAT_CN(CV_##tC#)
//// To get info on the type itself use
//// int bit_Depth  = CV_MAT_DEPTH_BITS(CV_##tC#)
//// int byte_Depth = CV_MAT_DEPTH_BYTES(CV_##tC#)
//// int channels = CV_MAT_CN(CV_##tC#)
//
//#define CV_AUTO_STEP  0x7fffffff
//#define CV_WHOLE_ARR  cvSlice( 0, 0x3fffffff )
//
//#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
//#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)
//#define CV_MAT_TYPE_MASK        (CV_DEPTH_MAX*CV_CN_MAX - 1)
//#define CV_MAT_TYPE(flags)      ((flags) & CV_MAT_TYPE_MASK)
//#define CV_MAT_CONT_FLAG_SHIFT  14
//#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
//#define CV_IS_MAT_CONT(flags)   ((flags) & CV_MAT_CONT_FLAG)
//#define CV_IS_CONT_MAT          CV_IS_MAT_CONT
//#define CV_SUBMAT_FLAG_SHIFT    15
//#define CV_SUBMAT_FLAG          (1 << CV_SUBMAT_FLAG_SHIFT)
//#define CV_IS_SUBMAT(flags)     ((flags) & CV_MAT_SUBMAT_FLAG)
//
//#ifndef MIN
//#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
//#endif
//
//#ifndef MAX
//#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
//#endif

/****************************************************************************************\
*          exchange-add operation for atomic operations on reference counters            *
\****************************************************************************************/

#ifdef CV_XADD
  // allow to use user-defined macro
#elif defined __GNUC__
#  if defined __clang__ && __clang_major__ >= 3 && !defined __ANDROID__ && !defined __EMSCRIPTEN__ && !defined(__CUDACC__)
#    ifdef __ATOMIC_ACQ_REL
#      define CV_XADD(addr, delta) __c11_atomic_fetch_add((_Atomic(int)*)(addr), delta, __ATOMIC_ACQ_REL)
#    else
#      define CV_XADD(addr, delta) __atomic_fetch_add((_Atomic(int)*)(addr), delta, 4)
#    endif
#  else
#    if defined __ATOMIC_ACQ_REL && !defined __clang__
       // version for gcc >= 4.7
#      define CV_XADD(addr, delta) (int)__atomic_fetch_add((unsigned*)(addr), (unsigned)(delta), __ATOMIC_ACQ_REL)
#    else
#      define CV_XADD(addr, delta) (int)__sync_fetch_and_add((unsigned*)(addr), (unsigned)(delta))
#    endif
#  endif
#elif defined _MSC_VER && !defined RC_INVOKED
#  include <intrin.h>
#  define CV_XADD(addr, delta) (int)_InterlockedExchangeAdd((long volatile*)addr, delta)
#else
   CV_INLINE CV_XADD(int* addr, int delta) { int tmp = *addr; *addr += delta; return tmp; }
#endif


/****************************************************************************************\
*                                  CV_NORETURN attribute                                 *
\****************************************************************************************/

#ifndef CV_NORETURN
#  if defined(__GNUC__)
#    define CV_NORETURN __attribute__((__noreturn__))
#  elif defined(_MSC_VER) && (_MSC_VER >= 1300)
#    define CV_NORETURN __declspec(noreturn)
#  else
#    define CV_NORETURN /* nothing by default */
#  endif
#endif



//! @}

#endif // OPENCV_CORE_CVDEF_H

#define CV_AUTO_STEP  0x7fffffff
#define CV_WHOLE_ARR  cvSlice( 0, 0x3fffffff )

#define CV_MAT_CN_MASK          ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags)        ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)
#define CV_MAT_TYPE_MASK        (CV_DEPTH_MAX*CV_CN_MAX - 1)
#define CV_MAT_TYPE(flags)      ((flags) & CV_MAT_TYPE_MASK)
#define CV_MAT_CONT_FLAG_SHIFT  14
#define CV_MAT_CONT_FLAG        (1 << CV_MAT_CONT_FLAG_SHIFT)
#define CV_IS_MAT_CONT(flags)   ((flags) & CV_MAT_CONT_FLAG)
#define CV_IS_CONT_MAT          CV_IS_MAT_CONT
#define CV_SUBMAT_FLAG_SHIFT    15
#define CV_SUBMAT_FLAG          (1 << CV_SUBMAT_FLAG_SHIFT)
#define CV_IS_SUBMAT(flags)     ((flags) & CV_MAT_SUBMAT_FLAG)

#define CV_MAGIC_MASK       0xFFFF0000
#define CV_MAT_MAGIC_VAL    0x42420000
#define CV_TYPE_NAME_MAT    "opencv-matrix"


/* Size of each channel item,
   0x124489 = 1000 0100 0100 0010 0010 0001 0001 ~ array of sizeof(arr_type_elem) */

/* 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
// Size of each channel item
#define CV_ELEM_SIZE1 CV_DEPTH_BYTES

// In case the channels are packed into fewer than one byte each we calculate : bits_used = channels * bits_per_channel
#define CV_ELEM_SIZE_BITS(type) ( CV_MAT_CN(type) * CV_MAT_DEPTH_BITS(type) )
// then bytes = Ceiling( bits_used / 8)
#define CV_ELEM_SIZE_BYTES(type) ( CV_MAT_CN(type) * CV_MAT_DEPTH_BYTES(type) )

#define CV_ELEM_SIZE CV_ELEM_SIZE_BYTES


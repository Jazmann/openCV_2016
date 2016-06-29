#ifndef OPENCV_CORE_HAL_INTERFACE_H
#define OPENCV_CORE_HAL_INTERFACE_H

//! @addtogroup core_hal_interface
//! @{

//! @name Return codes
//! @{
#define CV_HAL_ERROR_OK 0
#define CV_HAL_ERROR_NOT_IMPLEMENTED 1
#define CV_HAL_ERROR_UNKNOWN -1
//! @}

#ifdef __cplusplus
#include <cstddef>
#else
#include <stddef.h>
#include <stdbool.h>
#endif

//! @name Data types
//! primitive types
//! - schar  - signed 1 byte integer
//! - uchar  - unsigned 1 byte integer
//! - short  - signed 2 byte integer
//! - ushort - unsigned 2 byte integer
//! - int    - signed 4 byte integer
//! - uint   - unsigned 4 byte integer
//! - int64  - signed 8 byte integer
//! - uint64 - unsigned 8 byte integer
//! @{
#if !defined _MSC_VER && !defined __BORLANDC__
#  if defined __cplusplus && __cplusplus >= 201103L && !defined __APPLE__
#    include <cstdint>
     typedef std::uint32_t uint;
#  else
#    include <stdint.h>
     typedef uint32_t uint;
#  endif
#else
   typedef unsigned uint;
#endif

typedef signed char schar;

#ifndef __IPL_H__
   typedef unsigned char uchar;
   typedef unsigned short ushort;
#endif

#if defined _MSC_VER || defined __BORLANDC__
   typedef __int64 int64;
   typedef unsigned __int64 uint64;
#  define CV_BIG_INT(n)   n##I64
#  define CV_BIG_UINT(n)  n##UI64
#else
   typedef int64_t int64;
   typedef uint64_t uint64;
#  define CV_BIG_INT(n)   n##LL
#  define CV_BIG_UINT(n)  n##ULL
#endif


/****************************************************************************************\
 *                                  Matrix type (Mat)                                     *
 \****************************************************************************************/
// CV_CN_MAX reserves 9 bits for the channel numbers.
#define CV_CN_MAX     (1 << 9)
// CV_CN_SHIFT resrves the first 3 bits for the Data Types (CV_8U, etc)
#define CV_CN_SHIFT   3
// CV_DEPTH_MAX is the maximum number of Data types allowed.
#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)

// TYPE_TAB_ORDER puts a list of elements which are ordered by type into a list ordered by type numbering.
// The C++11 types are (uint8, int8, uint16, int16, uint32, int32, uint64, int64, float, double)
// The current type tab order which does not use all C++11 types is
// {CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F, CV_64F, CV_USRTYPE1}
//
// If CV_CN_SHIFT is increased to 4 bits then 16 types can be defined.
// This allows all 10 types to be included with 6 undefined types.
// (uint8, int8, uint16, int16, uint32, int32, uint64, int64, float, double, usr1, usr2, usr3, usr4, usr5, usr6)
// TYPE_TAB_ORDER would then be
// #define TYPE_TAB_ORDER(cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4) {cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4}
//
// For now the original order is preserved

#define TYPE_TAB_ORDER(cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4, cvUSRTYPE5, cvUSRTYPE6) {cv8U, cv8S, cv16U, cv16S, cv32S, cv32F, cv64F, cvUSRTYPE1}
#define D_TYPE_TAB_ORDER(cv8U, cv8S, cv16U, cv16S, cv32U, cv32S, cv64U, cv64S, cv32F, cv64F, cvUSRTYPE1, cvUSRTYPE2, cvUSRTYPE3, cvUSRTYPE4, cvUSRTYPE5, cvUSRTYPE6) {cv8U, cv8S, cv16U, cv16S, cv32S, cv32F, cv64F, cvUSRTYPE1}

#define CV_8U_EXISTS 1
#define CV_8S_EXISTS 1
#define CV_16U_EXISTS 1
#define CV_16S_EXISTS 1
#define CV_32U_EXISTS 0
#define CV_32S_EXISTS 1
#define CV_64U_EXISTS 0
#define CV_64S_EXISTS 0
#define CV_32F_EXISTS 1
#define CV_64F_EXISTS 1
#define CV_USRTYPE1_EXISTS 0
#define CV_USRTYPE2_EXISTS 0
#define CV_USRTYPE3_EXISTS 0
#define CV_USRTYPE4_EXISTS 0
#define CV_USRTYPE5_EXISTS 0
#define CV_USRTYPE6_EXISTS 0

#define IF_CV_8U_EXISTS(body) body
#define IF_CV_8S_EXISTS(body) body
#define IF_CV_16U_EXISTS(body) body
#define IF_CV_16S_EXISTS(body) body
#define IF_CV_32U_EXISTS(body)
#define IF_CV_32S_EXISTS(body) body
#define IF_CV_64U_EXISTS(body)
#define IF_CV_64S_EXISTS(body)
#define IF_CV_32F_EXISTS(body) body
#define IF_CV_64F_EXISTS(body) body
#define IF_CV_USRTYPE1_EXISTS(body)
#define IF_CV_USRTYPE2_EXISTS(body)
#define IF_CV_USRTYPE3_EXISTS(body)
#define IF_CV_USRTYPE4_EXISTS(body)
#define IF_CV_USRTYPE5_EXISTS(body)
#define IF_CV_USRTYPE6_EXISTS(body)

#define CV_8U   0
#define CV_8U_DEPTH_BITS_LOG2  3
#define CV_8U_DEPTH_BYTES_LOG2 0
#define CV_8U_TYPE             _STD uint8_t
#define CV_8U_MAX              UINT8_MAX
#define CV_8U_MIN              0

#define CV_8S   1
#define CV_8S_DEPTH_BITS_LOG2  3
#define CV_8S_DEPTH_BYTES_LOG2 0
#define CV_8S_TYPE             _STD int8_t
#define CV_8S_MAX              INT8_MAX
#define CV_8S_MIN              INT8_MIN

#define CV_16U  2
#define CV_16U_DEPTH_BITS_LOG2  4
#define CV_16U_DEPTH_BYTES_LOG2 1
#define CV_16U_TYPE             _STD uint16_t
#define CV_16U_MAX              UINT16_MAX
#define CV_16U_MIN              0

#define CV_16S  3
#define CV_16S_DEPTH_BITS_LOG2  4
#define CV_16S_DEPTH_BYTES_LOG2 1
#define CV_16S_TYPE             _STD int16_t
#define CV_16S_MAX              INT16_MAX
#define CV_16S_MIN              INT16_MIN

#define CV_32U  8
#define CV_32U_DEPTH_BITS_LOG2  5
#define CV_32U_DEPTH_BYTES_LOG2 2
#define CV_32U_TYPE             _STD uint32_t
#define CV_32U_MAX              UINT32_MAX
#define CV_32U_MIN              0

#define CV_32S  4
#define CV_32S_DEPTH_BITS_LOG2  5
#define CV_32S_DEPTH_BYTES_LOG2 2
#define CV_32S_TYPE             _STD int32_t
#define CV_32S_MAX              INT32_MAX
#define CV_32S_MIN              INT32_MIN

#define CV_64U  9
#define CV_64U_DEPTH_BITS_LOG2  6
#define CV_64U_DEPTH_BYTES_LOG2 3
#define CV_64U_TYPE             _STD uint64_t
#define CV_64U_MAX              UINT64_MAX
#define CV_64U_MIN              0

#define CV_64S  10
#define CV_64S_DEPTH_BITS_LOG2  6
#define CV_64S_DEPTH_BYTES_LOG2 3
#define CV_64S_TYPE             _STD int64_t
#define CV_64S_MAX              INT64_MAX
#define CV_64S_MIN              INT64_MIN

#define CV_32F  5
#define CV_32F_DEPTH_BITS_LOG2  5
#define CV_32F_DEPTH_BYTES_LOG2 2
#define CV_32F_TYPE             float
#define CV_32F_MAX              FLT_MAX
#define CV_32F_MIN              FLT_MIN

#define CV_64F  6
#define CV_64F_DEPTH_BITS_LOG2  6
#define CV_64F_DEPTH_BYTES_LOG2 3
#define CV_64F_TYPE             double
#define CV_64F_MAX              DBL_MAX
#define CV_64F_MIN              DBL_MIN

#define CV_USRTYPE1 7
#define CV_USRTYPE2 11
#define CV_USRTYPE3 12
#define CV_USRTYPE4 13
#define CV_USRTYPE5 14
#define CV_USRTYPE6 15

#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(type)    ((type) & (CV_MAT_DEPTH_MASK)) // CV_MAT_DEPTH(flags) applies the mask to the bits in flags.

#define CV_DEPTH_BYTES_MAGIC ( \
(CV_64F_DEPTH_BYTES_LOG2 << (CV_64F *2))|(CV_32F_DEPTH_BYTES_LOG2 << (CV_32F *2))|\
(CV_64S_DEPTH_BYTES_LOG2 << (CV_64S *2))|(CV_64U_DEPTH_BYTES_LOG2 << (CV_64U *2))|\
(CV_32S_DEPTH_BYTES_LOG2 << (CV_32S *2))|(CV_32U_DEPTH_BYTES_LOG2 << (CV_32U *2))|\
(CV_16S_DEPTH_BYTES_LOG2 << (CV_16S *2))|(CV_16U_DEPTH_BYTES_LOG2 << (CV_16U *2))|\
(CV_8S_DEPTH_BYTES_LOG2  << (CV_8S  *2))|(CV_8U_DEPTH_BYTES_LOG2  << (CV_8U  *2)) )

#define CV_MAT_DEPTH_BYTES(type) (1 << ( (CV_DEPTH_BYTES_MAGIC >> (CV_MAT_DEPTH(type)*2)) &3) )
#define CV_DEPTH_BYTES(type)     CV_MAT_DEPTH_BYTES(type) // Depreciated

#define CV_DEPTH_BITS_MAGIC ( \
(uint64_t(CV_64F_DEPTH_BITS_LOG2) << (CV_64F *3))|(uint64_t(CV_32F_DEPTH_BITS_LOG2) << (CV_32F *3))|\
(uint64_t(CV_64S_DEPTH_BITS_LOG2) << (CV_64S *3))|(uint64_t(CV_64U_DEPTH_BITS_LOG2) << (CV_64U *3))|\
(uint64_t(CV_32S_DEPTH_BITS_LOG2) << (CV_32S *3))|(uint64_t(CV_32U_DEPTH_BITS_LOG2) << (CV_32U *3))|\
(uint64_t(CV_16S_DEPTH_BITS_LOG2) << (CV_16S *3))|(uint64_t(CV_16U_DEPTH_BITS_LOG2) << (CV_16U *3))|\
(uint64_t(CV_8S_DEPTH_BITS_LOG2)  << (CV_8S  *3))|(uint64_t(CV_8U_DEPTH_BITS_LOG2)  << (CV_8U  *3)) )

#define CV_MAT_DEPTH_BITS(type)   (1 << ( (CV_DEPTH_BITS_MAGIC >> ((CV_MAT_DEPTH(type))*3)) & 7) )
#define CV_DEPTH_BITS(type)       CV_MAT_DEPTH_BITS(type) // Depreciated

// CV_MAKETYPE(depth,cn) generated an integer using the right-most CV_CN_SHIFT bits for the depth and the rest for the channels.

#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_MAKE_TYPE CV_MAKETYPE

#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
#define CV_8UC2 CV_MAKETYPE(CV_8U,2)
#define CV_8UC3 CV_MAKETYPE(CV_8U,3)
#define CV_8UC4 CV_MAKETYPE(CV_8U,4)
#define CV_8UC(n) CV_MAKETYPE(CV_8U,(n))

#define CV_8SC1 CV_MAKETYPE(CV_8S,1)
#define CV_8SC2 CV_MAKETYPE(CV_8S,2)
#define CV_8SC3 CV_MAKETYPE(CV_8S,3)
#define CV_8SC4 CV_MAKETYPE(CV_8S,4)
#define CV_8SC(n) CV_MAKETYPE(CV_8S,(n))

#define CV_16UC1 CV_MAKETYPE(CV_16U,1)
#define CV_16UC2 CV_MAKETYPE(CV_16U,2)
#define CV_16UC3 CV_MAKETYPE(CV_16U,3)
#define CV_16UC4 CV_MAKETYPE(CV_16U,4)
#define CV_16UC(n) CV_MAKETYPE(CV_16U,(n))

#define CV_16SC1 CV_MAKETYPE(CV_16S,1)
#define CV_16SC2 CV_MAKETYPE(CV_16S,2)
#define CV_16SC3 CV_MAKETYPE(CV_16S,3)
#define CV_16SC4 CV_MAKETYPE(CV_16S,4)
#define CV_16SC(n) CV_MAKETYPE(CV_16S,(n))

#define CV_32SC1 CV_MAKETYPE(CV_32S,1)
#define CV_32SC2 CV_MAKETYPE(CV_32S,2)
#define CV_32SC3 CV_MAKETYPE(CV_32S,3)
#define CV_32SC4 CV_MAKETYPE(CV_32S,4)
#define CV_32SC(n) CV_MAKETYPE(CV_32S,(n))

#define CV_32FC1 CV_MAKETYPE(CV_32F,1)
#define CV_32FC2 CV_MAKETYPE(CV_32F,2)
#define CV_32FC3 CV_MAKETYPE(CV_32F,3)
#define CV_32FC4 CV_MAKETYPE(CV_32F,4)
#define CV_32FC(n) CV_MAKETYPE(CV_32F,(n))

#define CV_64FC1 CV_MAKETYPE(CV_64F,1)
#define CV_64FC2 CV_MAKETYPE(CV_64F,2)
#define CV_64FC3 CV_MAKETYPE(CV_64F,3)
#define CV_64FC4 CV_MAKETYPE(CV_64F,4)
#define CV_64FC(n) CV_MAKETYPE(CV_64F,(n))

// To get back the information put into CV_MAKETYPE( depth_Type, cn) use
// int depth_Type = CV_MAT_DEPTH(CV_##tC#)
// int cn = CV_MAT_CN(CV_##tC#)
// To get info on the type itself use
// int bit_Depth  = CV_MAT_DEPTH_BITS(CV_##tC#)
// int byte_Depth = CV_MAT_DEPTH_BYTES(CV_##tC#)
// int channels = CV_MAT_CN(CV_##tC#)

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

#ifndef MIN
#  define    MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#  define    MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

#ifndef CV_MIN
#  define CV_MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef CV_MAX
#  define CV_MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

//! @}

//! @name Comparison operation
//! @sa cv::CmpTypes
//! @{
#define CV_HAL_CMP_EQ 0
#define CV_HAL_CMP_GT 1
#define CV_HAL_CMP_GE 2
#define CV_HAL_CMP_LT 3
#define CV_HAL_CMP_LE 4
#define CV_HAL_CMP_NE 5
//! @}

//! @name Border processing modes
//! @sa cv::BorderTypes
//! @{
#define CV_HAL_BORDER_CONSTANT 0
#define CV_HAL_BORDER_REPLICATE 1
#define CV_HAL_BORDER_REFLECT 2
#define CV_HAL_BORDER_WRAP 3
#define CV_HAL_BORDER_REFLECT_101 4
#define CV_HAL_BORDER_TRANSPARENT 5
#define CV_HAL_BORDER_ISOLATED 16
//! @}

//! @name DFT flags
//! @{
#define CV_HAL_DFT_INVERSE        1
#define CV_HAL_DFT_SCALE          2
#define CV_HAL_DFT_ROWS           4
#define CV_HAL_DFT_COMPLEX_OUTPUT 16
#define CV_HAL_DFT_REAL_OUTPUT    32
#define CV_HAL_DFT_TWO_STAGE      64
#define CV_HAL_DFT_STAGE_COLS    128
#define CV_HAL_DFT_IS_CONTINUOUS 512
#define CV_HAL_DFT_IS_INPLACE 1024
//! @}

//! @name SVD flags
//! @{
#define CV_HAL_SVD_NO_UV    1
#define CV_HAL_SVD_SHORT_UV 2
#define CV_HAL_SVD_MODIFY_A 4
#define CV_HAL_SVD_FULL_UV  8
//! @}

//! @name Gemm flags
//! @{
#define CV_HAL_GEMM_1_T 1
#define CV_HAL_GEMM_2_T 2
#define CV_HAL_GEMM_3_T 4
//! @}

//! @}

#endif

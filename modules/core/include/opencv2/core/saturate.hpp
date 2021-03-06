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
// Copyright (C) 2014, Itseez Inc., all rights reserved.
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

#ifndef OPENCV_CORE_SATURATE_HPP
#define OPENCV_CORE_SATURATE_HPP

#include "opencv2/core/cvdef.h"
#include "opencv2/core/fast_math.hpp"

namespace cv
{

//! @addtogroup core_utils
//! @{

/////////////// saturate_cast (used in image & signal processing) ///////////////////

/** @brief Template function for accurate conversion from one primitive type to another.

 The functions saturate_cast resemble the standard C++ cast operations, such as static_cast\<T\>()
 and others. They perform an efficient and accurate conversion from one primitive type to another
 (see the introduction chapter). saturate in the name means that when the input value v is out of the
 range of the target type, the result is not formed just by taking low bits of the input, but instead
 the value is clipped. For example:
 @code
 uchar a = saturate_cast<uchar>(-100); // a = 0 (UCHAR_MIN)
 short b = saturate_cast<short>(33333.33333); // b = 32767 (SHRT_MAX)
 @endcode
 Such clipping is done when the target type is unsigned char , signed char , unsigned short or
 signed short . For 32-bit integers, no clipping is done.

 When the parameter is a floating-point value and the target type is an integer (8-, 16- or 32-bit),
 the floating-point value is first rounded to the nearest integer and then clipped if needed (when
 the target type is 8- or 16-bit).

 This operation is used in the simplest or most complex image processing functions in OpenCV.

 @param v Function parameter.
 @sa add, subtract, multiply, divide, Mat::convertTo
 */

template<typename _Tp> static inline _Tp saturate_cast(CV_8U_TYPE v)    { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_8S_TYPE v)    { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_16U_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_16S_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_32U_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_32S_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_64U_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_64S_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_32F_TYPE v)   { return _Tp(v); }
/** @overload */
template<typename _Tp> static inline _Tp saturate_cast(CV_64F_TYPE v)   { return _Tp(v); }

template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_8S_TYPE v)  { return (CV_8U_TYPE)std::max((CV_32S_TYPE)v, 0); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_16U_TYPE v) { return (CV_8U_TYPE)std::min((unsigned)v, (unsigned)CV_8U_MAX); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_16S_TYPE v) { return (CV_8U_TYPE)std::max((int)v, 0); }

template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_32S_TYPE v) { return (CV_8U_TYPE)((unsigned)v <= CV_8U_MAX ? v : v > 0 ? CV_8U_MAX : 0); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_32U_TYPE v) { return (CV_8U_TYPE)std::min(v, (unsigned)CV_8U_MAX); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_32F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_8U_TYPE>(iv); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_64F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_8U_TYPE>(iv); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_64U_TYPE v) { return (CV_8U_TYPE)std::min(v, (CV_64U_TYPE)CV_8U_MAX); }
template<> inline CV_8U_TYPE saturate_cast<CV_8U_TYPE>(CV_64S_TYPE v) { return (CV_8U_TYPE)(v <= (CV_64S_TYPE)CV_8U_MAX ? (v > 0 ? v : 0) : CV_8U_MAX); }

template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_8U_TYPE v)  { return (CV_8S_TYPE)std::min((CV_32S_TYPE)v, CV_8S_MAX); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_16U_TYPE v) { return (CV_8S_TYPE)std::min((CV_32U_TYPE)v, (CV_32U_TYPE)CV_8S_MAX); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_32S_TYPE v) { return (CV_8S_TYPE)((CV_32U_TYPE)(v-CV_8S_MIN) <= (CV_32U_TYPE)CV_8U_MAX ? v : v > 0 ? CV_8S_MAX : CV_8S_MIN); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_16S_TYPE v) { return saturate_cast<CV_8S_TYPE>((CV_32S_TYPE)v); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_32U_TYPE v) { return (CV_8S_TYPE)std::min(v, (CV_32U_TYPE)CV_8S_MAX); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_32F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_8S_TYPE>(iv); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_64F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_8S_TYPE>(iv); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_64U_TYPE v) { return (CV_8S_TYPE)std::min(v, (CV_64U_TYPE)CV_8S_MAX); }
template<> inline CV_8S_TYPE saturate_cast<CV_8S_TYPE>(CV_64S_TYPE v) { return (CV_8S_TYPE)(v>0 ? std::min(v, (CV_64S_TYPE)CV_8S_MAX) : std::max(v,(CV_64S_TYPE)CV_8S_MIN)); }

template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_8S_TYPE v)  { return (CV_16U_TYPE)std::max((CV_32S_TYPE)v, 0); }
template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_16S_TYPE v) { return (CV_16U_TYPE)std::max((CV_32S_TYPE)v, 0); }
template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_32S_TYPE v) { return (CV_16U_TYPE)((CV_32U_TYPE)v <= (CV_32U_TYPE)CV_16U_MAX ? v : v > 0 ? CV_16U_MAX : 0); }
template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_32U_TYPE v) { return (CV_16U_TYPE)std::min(v, (CV_32U_TYPE)CV_16U_MAX); }
template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_32F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_16U_TYPE>(iv); }
template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_64F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_16U_TYPE>(iv); }

template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_64U_TYPE v) { return (CV_16U_TYPE)std::min(v, (CV_64U_TYPE)CV_16U_MAX); }
template<> inline CV_16U_TYPE saturate_cast<CV_16U_TYPE>(CV_64S_TYPE v) { return (CV_16U_TYPE)(v <= (CV_64S_TYPE)CV_16U_MAX ? (v > 0 ? v : 0) : CV_16U_MAX); }

template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_16U_TYPE v) { return (CV_16S_TYPE)std::min((CV_32S_TYPE)v, CV_16S_MAX); }
template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_32S_TYPE v) { return (CV_16S_TYPE)((CV_32U_TYPE)(v - CV_16S_MIN) <= (CV_32U_TYPE)CV_16U_MAX ? v : v > 0 ? CV_16S_MAX : CV_16S_MIN); }
template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_32U_TYPE v) { return (CV_16S_TYPE)std::min(v, (CV_32U_TYPE)CV_16S_MAX); }
template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_32F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_16S_TYPE>(iv); }
template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_64F_TYPE v) { CV_32S_TYPE iv = cvRound(v); return saturate_cast<CV_16S_TYPE>(iv); }

template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_64U_TYPE v) { return (CV_16S_TYPE)std::min(v, (CV_64U_TYPE)CV_16S_MAX); }
template<> inline CV_16S_TYPE saturate_cast<CV_16S_TYPE>(CV_64S_TYPE v) { return (CV_16S_TYPE)(v>0 ? std::min(v, (CV_64S_TYPE)CV_16S_MAX) : std::max(v,(CV_64S_TYPE)CV_16S_MIN)); }

template<> inline CV_32U_TYPE saturate_cast<CV_32U_TYPE>(CV_32F_TYPE v) { return cvRound(v); }
template<> inline CV_32U_TYPE saturate_cast<CV_32U_TYPE>(CV_64F_TYPE v) { return cvRound(v); }

template<> inline CV_32U_TYPE saturate_cast<CV_32U_TYPE>(CV_64U_TYPE v) { return (CV_32U_TYPE)std::min(v, (CV_64U_TYPE)CV_32U_MAX); }
template<> inline CV_32U_TYPE saturate_cast<CV_32U_TYPE>(CV_64S_TYPE v) { return (CV_32U_TYPE)(v <= (CV_64S_TYPE)CV_32U_MAX ? (v > 0 ? v : 0) : CV_32U_MAX); }

template<> inline CV_32S_TYPE saturate_cast<CV_32S_TYPE>(CV_32F_TYPE v) { return cvRound(v); }
template<> inline CV_32S_TYPE saturate_cast<CV_32S_TYPE>(CV_64F_TYPE v) { return cvRound(v); }

template<> inline CV_32S_TYPE saturate_cast<CV_32S_TYPE>(CV_64U_TYPE v){ return (CV_32S_TYPE)std::min(v, (CV_64U_TYPE)CV_32S_MAX); }
template<> inline CV_32S_TYPE saturate_cast<CV_32S_TYPE>(CV_64S_TYPE v){ return (CV_32S_TYPE)(v>0 ? std::min(v, (CV_64S_TYPE)CV_32S_MAX) : std::max(v,(CV_64S_TYPE)CV_32S_MIN)); }

template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_8U_TYPE v) { return (CV_64U_TYPE)v; }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_8S_TYPE v) { return (CV_64U_TYPE)(v > 0 ? v : 0); }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_16U_TYPE v){ return (CV_64U_TYPE)v; }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_16S_TYPE v){ return (CV_64U_TYPE)(v > 0 ? v : 0); }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_32U_TYPE v){ return (CV_64U_TYPE)v; }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_32S_TYPE v){ return (CV_64U_TYPE)(v > 0 ? v : 0);  }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_64U_TYPE v){ return v; }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_64S_TYPE v){ return (CV_64U_TYPE)(v > 0 ? v : 0); }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_32F_TYPE v){ return (CV_64U_TYPE) cv64Round(v); }
template<> inline CV_64U_TYPE saturate_cast<CV_64U_TYPE>(CV_64F_TYPE v){ return (CV_64U_TYPE) cv64Round(v); }

template<> inline CV_64S_TYPE saturate_cast<CV_64S_TYPE>(CV_64U_TYPE v){ return (CV_64S_TYPE)std::min(v, (CV_64U_TYPE)CV_64S_MAX); }
template<> inline CV_64S_TYPE saturate_cast<CV_64S_TYPE>(CV_32F_TYPE v){ return (CV_64S_TYPE) cv64Round(v); }
template<> inline CV_64S_TYPE saturate_cast<CV_64S_TYPE>(CV_64F_TYPE v){ return (CV_64S_TYPE) cv64Round(v); }

//! @}

} // cv

#endif // OPENCV_CORE_SATURATE_HPP

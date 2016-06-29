//
//  cvtraits.hpp
//  OpenCV
//
//  Created by Jasper Shemilt on 11/02/2016.
//
//

#ifndef cvtraits_h
#define cvtraits_h

#ifndef __cplusplus
#  error cvtraits.hpp header must be compiled as C++
#endif

#ifndef CV_MIN
#  define CV_MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef CV_MAX
#  define CV_MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

template<int t> struct cv_Data_Type{
    using type  = CV_8U_TYPE;
    using sType = CV_8S_TYPE;
    using uType = CV_8U_TYPE;
    const static int channelType = CV_MAT_DEPTH_MASK & t;
    const static int channels = CV_MAT_CN(t);
    const static int dataType = t;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(t);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(t);
    constexpr static type max  = cv_Data_Type<CV_MAT_DEPTH(t)>::max;
    constexpr static type min  = cv_Data_Type<CV_MAT_DEPTH(t)>::min;
    constexpr const static char fmt[5] = { '%', 'i', '\0', ' ', ' ' };
};

template<> struct cv_Data_Type<CV_8U>{
    using type  = CV_8U_TYPE;
    using sType = CV_8S_TYPE;
    using uType = CV_8U_TYPE;
    const static int channelType = CV_8U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_8U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_8U);
    constexpr static type max  = 0xFF;
    constexpr static type min  = 0;
    constexpr const static char fmt[5] = { '%', 'h', 'h', 'u', '\0' }; // "hhu"
};

template<> struct cv_Data_Type<CV_8S>{
    using type  = CV_8S_TYPE;
    using sType = CV_8S_TYPE;
    using uType = CV_8U_TYPE;
    const static int channelType = CV_8S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_8S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_8S);
    constexpr static type max  =  0x7F;
    constexpr static type min  = -0x7F;
    constexpr const static char fmt[5] = { '%', 'h', 'h', 'i', '\0' }; //"hhi";
};

template<> struct cv_Data_Type<CV_16U>{
    using type  = CV_16U_TYPE;
    using sType = CV_16S_TYPE;
    using uType = CV_16U_TYPE;
    const static int channelType = CV_16U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_16U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_16U);
    constexpr static type max  = 0xFFFF;
    constexpr static type min  = 0;
    constexpr const static char fmt[5] = { '%', 'h', 'u', '\0', ' ' }; //"hu";
};

template<> struct cv_Data_Type<CV_16S>{
    using type  = CV_16S_TYPE;
    using sType = CV_16S_TYPE;
    using uType = CV_16U_TYPE;
    const static int channelType = CV_16S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_16S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_16S);
    constexpr static type max  =  0x7FFF;
    constexpr static type min  = -0x7FFF;
    constexpr const static char fmt[5] = { '%', 'h', 'i', '\0', ' ' }; //"hi";
};

template<> struct cv_Data_Type<CV_32U>{
    using type  = CV_32U_TYPE;
    using sType = CV_32S_TYPE;
    using uType = CV_32U_TYPE;
    const static int channelType = CV_32U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_32U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_32U);
    constexpr static type max  = 0xFFFFFFFF;
    constexpr static type min  = 0;
    constexpr const static char fmt[5] = { '%', 'u', '\0', ' ', ' ' }; // "u";
};

template<> struct cv_Data_Type<CV_32S>{
    using type  = CV_32S_TYPE;
    using sType = CV_32S_TYPE;
    using uType = CV_32U_TYPE;
    const static int channelType = CV_32S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_32S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_32S);
    constexpr static type max  =  0x7FFFFFFF;
    constexpr static type min  = -0x7FFFFFFF;
    constexpr const static char fmt[5] = { '%', 'i', '\0', ' ', ' ' }; // "i";
};

template<> struct cv_Data_Type<CV_64U>{
    using type  = CV_64U_TYPE;
    using sType = CV_64S_TYPE;
    using uType = CV_64U_TYPE;
    const static int channelType = CV_64U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_64U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_64U);
    constexpr static type max  = 0xFFFFFFFFFFFFFFFF;
    constexpr static type min  = 0;
    constexpr const static char fmt[5] = { '%', 'l', 'l', 'u', '\0' }; //"llu";
};

template<> struct cv_Data_Type<CV_64S>{
    using type  = CV_64S_TYPE;
    using sType = CV_64S_TYPE;
    using uType = CV_64U_TYPE;
    const static int channelType = CV_64S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_64S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_64S);
    constexpr static type max  =  0x7FFFFFFFFFFFFFFF;;
    constexpr static type min  = -0x7FFFFFFFFFFFFFFF;
    constexpr const static char fmt[5] = { '%', 'l', 'l', 'i', '\0' }; //"lli";
};

template<> struct cv_Data_Type<CV_32F>{
    using type  = CV_32F_TYPE;
    using sType = CV_32F_TYPE;
    using uType = CV_32F_TYPE;
    const static int channelType = CV_32F;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_32F);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_32F);
    constexpr static type max  = CV_32F_MAX;
    constexpr static type min  = CV_32F_MIN;
    constexpr const static char fmt[5] = { '%', 'f', '\0', ' ', ' ' }; // "f";
};

template<> struct cv_Data_Type<CV_64F>{
    using type  = CV_64F_TYPE;
    using sType = CV_64F_TYPE;
    using uType = CV_64F_TYPE;
    const static int channelType = CV_64F;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_64F);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_64F);
    constexpr static type max  = CV_64F_MAX;
    constexpr static type min  = CV_64F_MIN;
    constexpr const static char fmt[5] = { '%', 'f', '\0', ' ', ' ' }; // "f";
};

template<int t1, int t2> struct cv_Work_Type{
    using type = typename cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::type;
    const static int channelType = cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::channelType;
    const static int bitDepth  = cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::bitDepth;
    const static int byteDepth = cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::byteDepth;
    constexpr static type max  = cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::max;
    constexpr static type min  = cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::min;
    const char* fmt = cv_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::fmt;
};

template<> struct cv_Work_Type<CV_8U, CV_8U> : cv_Data_Type<CV_16U>{};
template<> struct cv_Work_Type<CV_8U,CV_16U> : cv_Data_Type<CV_32U>{};
template<> struct cv_Work_Type<CV_8U,CV_32U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_8U,CV_64U> : cv_Data_Type<CV_64U>{};

template<> struct cv_Work_Type<CV_8U, CV_8S> : cv_Data_Type<CV_16S>{};
template<> struct cv_Work_Type<CV_8U,CV_16S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Work_Type<CV_8U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_8U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_8S, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Work_Type<CV_8S, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_8S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_8S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_16U, CV_8U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_16U,CV_16U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_16U,CV_32U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_16U,CV_64U> : cv_Data_Type<CV_64U>{};

template<> struct cv_Work_Type<CV_16U, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16U,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_16U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_16S, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Work_Type<CV_16S, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_16S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_16S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_32U, CV_8U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_32U,CV_16U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_32U,CV_32U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_32U,CV_64U> : cv_Data_Type<CV_64U>{};

template<> struct cv_Work_Type<CV_32U, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32U,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_32S, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Work_Type<CV_32S, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_32S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_64U, CV_8U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_64U,CV_16U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_64U,CV_32U> : cv_Data_Type<CV_64U>{};
template<> struct cv_Work_Type<CV_64U,CV_64U> : cv_Data_Type<CV_64U>{};

template<> struct cv_Work_Type<CV_64U, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64U,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_64S, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Work_Type<CV_64S, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Work_Type<CV_64S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_32F, CV_8U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_16U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_32U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_64U> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_32F, CV_8S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_16S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_32S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_64S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_32F,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_64F, CV_8U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_16U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_32U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_64U> : cv_Data_Type<CV_64F>{};

template<> struct cv_Work_Type<CV_64F, CV_8S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_16S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_32S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_64S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Work_Type<CV_64F,CV_64F> : cv_Data_Type<CV_64F>{};

//


template struct cv_Work_Type<CV_8U, CV_8U>;
template struct cv_Work_Type<CV_8U,CV_16U>;
template struct cv_Work_Type<CV_8U,CV_32U>;
template struct cv_Work_Type<CV_8U,CV_64U>;

template struct cv_Work_Type<CV_8U, CV_8S>;
template struct cv_Work_Type<CV_8U,CV_16S>;
template struct cv_Work_Type<CV_8U,CV_32S>;
template struct cv_Work_Type<CV_8U,CV_64S>;
template struct cv_Work_Type<CV_8U,CV_32F>;
template struct cv_Work_Type<CV_8U,CV_64F>;

template struct cv_Work_Type<CV_16U, CV_8U>;
template struct cv_Work_Type<CV_16U,CV_16U>;
template struct cv_Work_Type<CV_16U,CV_32U>;
template struct cv_Work_Type<CV_16U,CV_64U>;

template struct cv_Work_Type<CV_16U, CV_8S>;
template struct cv_Work_Type<CV_16U,CV_16S>;
template struct cv_Work_Type<CV_16U,CV_32S>;
template struct cv_Work_Type<CV_16U,CV_64S>;
template struct cv_Work_Type<CV_16U,CV_32F>;
template struct cv_Work_Type<CV_16U,CV_64F>;

template struct cv_Work_Type<CV_32U, CV_8U>;
template struct cv_Work_Type<CV_32U,CV_16U>;
template struct cv_Work_Type<CV_32U,CV_32U>;
template struct cv_Work_Type<CV_32U,CV_64U>;

template struct cv_Work_Type<CV_32U, CV_8S>;
template struct cv_Work_Type<CV_32U,CV_16S>;
template struct cv_Work_Type<CV_32U,CV_32S>;
template struct cv_Work_Type<CV_32U,CV_64S>;
template struct cv_Work_Type<CV_32U,CV_32F>;
template struct cv_Work_Type<CV_32U,CV_64F>;

template struct cv_Work_Type<CV_64U, CV_8U>;
template struct cv_Work_Type<CV_64U,CV_16U>;
template struct cv_Work_Type<CV_64U,CV_32U>;
template struct cv_Work_Type<CV_64U,CV_64U>;

template struct cv_Work_Type<CV_64U, CV_8S>;
template struct cv_Work_Type<CV_64U,CV_16S>;
template struct cv_Work_Type<CV_64U,CV_32S>;
template struct cv_Work_Type<CV_64U,CV_64S>;
template struct cv_Work_Type<CV_64U,CV_32F>;
template struct cv_Work_Type<CV_64U,CV_64F>;

template struct cv_Work_Type<CV_32F, CV_8U>;
template struct cv_Work_Type<CV_32F,CV_16U>;
template struct cv_Work_Type<CV_32F,CV_32U>;
template struct cv_Work_Type<CV_32F,CV_64U>;

template struct cv_Work_Type<CV_32F, CV_8S>;
template struct cv_Work_Type<CV_32F,CV_16S>;
template struct cv_Work_Type<CV_32F,CV_32S>;
template struct cv_Work_Type<CV_32F,CV_64S>;
template struct cv_Work_Type<CV_32F,CV_32F>;
template struct cv_Work_Type<CV_32F,CV_64F>;

template struct cv_Work_Type<CV_64F, CV_8U>;
template struct cv_Work_Type<CV_64F,CV_16U>;
template struct cv_Work_Type<CV_64F,CV_32U>;
template struct cv_Work_Type<CV_64F,CV_64U>;

template struct cv_Work_Type<CV_64F, CV_8S>;
template struct cv_Work_Type<CV_64F,CV_16S>;
template struct cv_Work_Type<CV_64F,CV_32S>;
template struct cv_Work_Type<CV_64F,CV_64S>;
template struct cv_Work_Type<CV_64F,CV_32F>;
template struct cv_Work_Type<CV_64F,CV_64F>;

// Signed Working types

template<int t1, int t2> struct cv_Signed_Work_Type{
    using type = typename cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::type;
    const static int channelType = cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::channelType;
    const static int bitDepth  = cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::bitDepth;
    const static int byteDepth = cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::byteDepth;
    constexpr static type max  = cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::max;
    constexpr static type min  = cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::min;
    const char* fmt = cv_Signed_Work_Type<CV_MAT_DEPTH(t1), CV_MAT_DEPTH(t2)>::fmt;
};

template<> struct cv_Signed_Work_Type<CV_8U, CV_8U> : cv_Data_Type<CV_16S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_16U> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_8U, CV_8S> : cv_Data_Type<CV_16S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_16S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_8U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_8S, CV_8U> : cv_Data_Type<CV_16S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_16U> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_8S, CV_8S> : cv_Data_Type<CV_16S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_16S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_8S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_16U, CV_8U> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_16U, CV_8S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_16S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_16U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_16S, CV_8U> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_16U> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_16S, CV_8S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_16S> : cv_Data_Type<CV_32S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_16S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_32U, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_32U, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_32S, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_32S, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_64U, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_64U, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64U,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_64S, CV_8U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_16U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_32U> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_64U> : cv_Data_Type<CV_64S>{};

template<> struct cv_Signed_Work_Type<CV_64S, CV_8S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_16S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_32S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_64S> : cv_Data_Type<CV_64S>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64S,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_32F, CV_8U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_16U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_32U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_64U> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_32F, CV_8S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_16S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_32S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_64S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_32F,CV_64F> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_64F, CV_8U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_16U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_32U> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_64U> : cv_Data_Type<CV_64F>{};

template<> struct cv_Signed_Work_Type<CV_64F, CV_8S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_16S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_32S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_64S> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_32F> : cv_Data_Type<CV_64F>{};
template<> struct cv_Signed_Work_Type<CV_64F,CV_64F> : cv_Data_Type<CV_64F>{};

// Instantiate templates

template struct cv_Signed_Work_Type<CV_8U, CV_8U>;
template struct cv_Signed_Work_Type<CV_8U,CV_16U>;
template struct cv_Signed_Work_Type<CV_8U,CV_32U>;
template struct cv_Signed_Work_Type<CV_8U,CV_64U>;

template struct cv_Signed_Work_Type<CV_8U, CV_8S>;
template struct cv_Signed_Work_Type<CV_8U,CV_16S>;
template struct cv_Signed_Work_Type<CV_8U,CV_32S>;
template struct cv_Signed_Work_Type<CV_8U,CV_64S>;
template struct cv_Signed_Work_Type<CV_8U,CV_32F>;
template struct cv_Signed_Work_Type<CV_8U,CV_64F>;

template struct cv_Signed_Work_Type<CV_16U, CV_8U>;
template struct cv_Signed_Work_Type<CV_16U,CV_16U>;
template struct cv_Signed_Work_Type<CV_16U,CV_32U>;
template struct cv_Signed_Work_Type<CV_16U,CV_64U>;

template struct cv_Signed_Work_Type<CV_16U, CV_8S>;
template struct cv_Signed_Work_Type<CV_16U,CV_16S>;
template struct cv_Signed_Work_Type<CV_16U,CV_32S>;
template struct cv_Signed_Work_Type<CV_16U,CV_64S>;
template struct cv_Signed_Work_Type<CV_16U,CV_32F>;
template struct cv_Signed_Work_Type<CV_16U,CV_64F>;

template struct cv_Signed_Work_Type<CV_32U, CV_8U>;
template struct cv_Signed_Work_Type<CV_32U,CV_16U>;
template struct cv_Signed_Work_Type<CV_32U,CV_32U>;
template struct cv_Signed_Work_Type<CV_32U,CV_64U>;

template struct cv_Signed_Work_Type<CV_32U, CV_8S>;
template struct cv_Signed_Work_Type<CV_32U,CV_16S>;
template struct cv_Signed_Work_Type<CV_32U,CV_32S>;
template struct cv_Signed_Work_Type<CV_32U,CV_64S>;
template struct cv_Signed_Work_Type<CV_32U,CV_32F>;
template struct cv_Signed_Work_Type<CV_32U,CV_64F>;

template struct cv_Signed_Work_Type<CV_64U, CV_8U>;
template struct cv_Signed_Work_Type<CV_64U,CV_16U>;
template struct cv_Signed_Work_Type<CV_64U,CV_32U>;
template struct cv_Signed_Work_Type<CV_64U,CV_64U>;

template struct cv_Signed_Work_Type<CV_64U, CV_8S>;
template struct cv_Signed_Work_Type<CV_64U,CV_16S>;
template struct cv_Signed_Work_Type<CV_64U,CV_32S>;
template struct cv_Signed_Work_Type<CV_64U,CV_64S>;
template struct cv_Signed_Work_Type<CV_64U,CV_32F>;
template struct cv_Signed_Work_Type<CV_64U,CV_64F>;

template struct cv_Signed_Work_Type<CV_32F, CV_8U>;
template struct cv_Signed_Work_Type<CV_32F,CV_16U>;
template struct cv_Signed_Work_Type<CV_32F,CV_32U>;
template struct cv_Signed_Work_Type<CV_32F,CV_64U>;

template struct cv_Signed_Work_Type<CV_32F, CV_8S>;
template struct cv_Signed_Work_Type<CV_32F,CV_16S>;
template struct cv_Signed_Work_Type<CV_32F,CV_32S>;
template struct cv_Signed_Work_Type<CV_32F,CV_64S>;
template struct cv_Signed_Work_Type<CV_32F,CV_32F>;
template struct cv_Signed_Work_Type<CV_32F,CV_64F>;

template struct cv_Signed_Work_Type<CV_64F, CV_8U>;
template struct cv_Signed_Work_Type<CV_64F,CV_16U>;
template struct cv_Signed_Work_Type<CV_64F,CV_32U>;
template struct cv_Signed_Work_Type<CV_64F,CV_64U>;

template struct cv_Signed_Work_Type<CV_64F, CV_8S>;
template struct cv_Signed_Work_Type<CV_64F,CV_16S>;
template struct cv_Signed_Work_Type<CV_64F,CV_32S>;
template struct cv_Signed_Work_Type<CV_64F,CV_64S>;
template struct cv_Signed_Work_Type<CV_64F,CV_32F>;
template struct cv_Signed_Work_Type<CV_64F,CV_64F>;

template<int cv_data_type> using cv_Type = typename cv_Data_Type<cv_data_type>::type;

// Don't use cv_Data_Type directly; use Data_Type which works for both types and types with channels.

namespace cv {
    template<int t> struct Data_Type : cv_Data_Type<CV_MAT_DEPTH(t)>{
        constexpr static int channels  = CV_MAT_CN(t);
        const static int dataType = t;
    };
    template<int t1,int t2> struct Work_Type : cv_Work_Type<CV_MAT_DEPTH(t1),CV_MAT_DEPTH(t2)>{
        constexpr static int channels  = CV_MAX(CV_MAT_CN(t1),CV_MAT_CN(t2));
        const static int dataType = CV_MAKETYPE((cv_Work_Type<CV_MAT_DEPTH(t1),CV_MAT_DEPTH(t2)>::channelType), (CV_MAX(CV_MAT_CN(t1), CV_MAT_CN(t2))) );
    };
    template<int t1,int t2> struct Signed_Work_Type : cv_Signed_Work_Type<CV_MAT_DEPTH(t1),CV_MAT_DEPTH(t2)>{
        constexpr static int channels  = CV_MAX(CV_MAT_CN(t1),CV_MAT_CN(t2));
        const static int dataType = CV_MAKETYPE((cv_Signed_Work_Type<CV_MAT_DEPTH(t1),CV_MAT_DEPTH(t2)>::channelType), (CV_MAX(CV_MAT_CN(t1), CV_MAT_CN(t2))) );
    };
    

}

#endif /* cvtraits_h */

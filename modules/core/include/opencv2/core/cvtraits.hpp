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
#  error core.hpp header must be compiled as C++
#endif

template<int t> struct cv_Data_Type{
    using type = unsigned char;
    const static int dataType = t;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(t);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(t);
    //    constexpr static type max  = CV_MAT_MAX(t);
    //    constexpr static type min  = CV_MAT_MIN(t);
    
};
template<> struct cv_Data_Type<CV_8U>{
    using type = CV_8U_TYPE;
    const static int dataType = CV_8U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_8U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_8U);
    constexpr static type max  = CV_8U_MAX;
    constexpr static type min  = CV_8U_MIN;
    const char* fmt = "hhu";
};
template<> struct cv_Data_Type<CV_8S>{
    using type = CV_8S_TYPE;
    const static int dataType = CV_8S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_8S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_8S);
    constexpr static type max  = CV_8S_MAX;
    constexpr static type min  = CV_8S_MIN;
    const char* fmt = "hhi";
};
template<> struct cv_Data_Type<CV_16U>{
    using type = CV_16U_TYPE;
    const static int dataType = CV_16U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_16U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_16U);
    constexpr static type max  = CV_16U_MAX;
    constexpr static type min  = CV_16U_MIN;
    const char* fmt = "hu";
};
template<> struct cv_Data_Type<CV_16S>{
    using type = CV_16S_TYPE;
    const static int dataType = CV_16S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_16S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_16S);
    constexpr static type max  = CV_16S_MAX;
    constexpr static type min  = CV_16S_MIN;
    const char* fmt = "hi";
};
template<> struct cv_Data_Type<CV_32U>{
    using type = CV_32U_TYPE;
    const static int dataType = CV_32U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_32U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_32U);
    constexpr static type max  = CV_32U_MAX;
    constexpr static type min  = CV_32U_MIN;
    const char* fmt = "u";
};
template<> struct cv_Data_Type<CV_32S>{
    using type = CV_32S_TYPE;
    const static int dataType = CV_32S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_32S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_32S);
    constexpr static type max  = CV_32S_MAX;
    constexpr static type min  = CV_32S_MIN;
    const char* fmt = "i";
};
template<> struct cv_Data_Type<CV_64U>{
    using type = CV_64U_TYPE;
    const static int dataType = CV_64U;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_64U);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_64U);
    constexpr static type max  = CV_64U_MAX;
    constexpr static type min  = CV_64U_MIN;
    const char* fmt = "llu";
};
template<> struct cv_Data_Type<CV_64S>{
    using type = CV_64S_TYPE;
    const static int dataType = CV_64S;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_64S);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_64S);
    const static type max  = CV_64S_MAX;
    const static type min  = CV_64S_MIN;
    const char* fmt = "lli";
};
template<> struct cv_Data_Type<CV_32F>{
    using type = CV_32F_TYPE;
    const static int dataType = CV_32F;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_32F);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_32F);
    constexpr static type max  = CV_32F_MAX;
    constexpr static type min  = CV_32F_MIN;
    const char* fmt = "f";
};
template<> struct cv_Data_Type<CV_64F>{
    using type = CV_64F_TYPE;
    const static int dataType = CV_64F;
    const static int bitDepth  = CV_MAT_DEPTH_BITS(CV_64F);
    const static int byteDepth = CV_MAT_DEPTH_BYTES(CV_64F);
    constexpr static type max  = CV_64F_MAX;
    constexpr static type min  = CV_64F_MIN;
    const char* fmt = "f";
};

template<int t1, int t2> struct cv_Work_Type;

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

template<int t1, int t2> struct cv_Signed_Work_Type;

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
    };
    template<int t1,int t2> struct Work_Type : cv_Work_Type<CV_MAT_DEPTH(t1),CV_MAT_DEPTH(t2)>{
        constexpr static int channels  = CV_MAT_CN(t1) + CV_MAT_CN(t2);
    };
    template<int t1,int t2> struct Signed_Work_Type : cv_Signed_Work_Type<CV_MAT_DEPTH(t1),CV_MAT_DEPTH(t2)>{
        constexpr static int channels  = CV_MAT_CN(t1) + CV_MAT_CN(t2);
    };
    
    
}

#endif /* cvtraits_h */

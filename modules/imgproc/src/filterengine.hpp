/*
By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#ifndef __OPENCV_IMGPROC_FILTERENGINE_HPP__
#define __OPENCV_IMGPROC_FILTERENGINE_HPP__

#include "opencv2/core/cvtraits.hpp"

namespace cv
{

//! type of the kernel
enum
{
    KERNEL_GENERAL      = 0, // the kernel is generic. No any type of symmetry or other properties.
    KERNEL_SYMMETRICAL  = 1, // kernel[i] == kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_ASYMMETRICAL = 2, // kernel[i] == -kernel[ksize-i-1] , and the anchor is at the center
    KERNEL_SMOOTH       = 4, // all the kernel elements are non-negative and summed to 1
    KERNEL_INTEGER      = 8  // all the kernel coefficients are integer numbers
};
    
    template<int src_t, int dst_t> class CV_EXPORTS distributeErfParameters
    {
        public :
        using srcInfo = cv::Data_Type<src_t>;
        using dstInfo = cv::Data_Type<dst_t>;
        using srcType = typename cv::Data_Type<src_t>::type;
        using dstType = typename cv::Data_Type<dst_t>::type;
        using wrkInfo = cv::Work_Type<src_t, dst_t>;
        using wrkType = typename cv::Work_Type<src_t, dst_t>::type;
        const srcType lookUpTableMax = 255;
//        const srcType nonLinearMin = 3; // Less than this is is not worth keeping the error function at all.
        
        srcType sMin, sMax, sRange;
        dstType dMin, dMax, dRange;
        // The distribution is described using 2 numbers which are related to the center and the standard deviation.
        // If the center is in the range 0:1 then it is assumed that both the standard deviation and the
        // center are specified in the 0:1 range. The type of the number also sets the choice of range.
        // Distinguishing between the standard deviation s and the related parameter g is more difficult.
        // +ve values are taken to be s
        // -ve values are taken to be g
        srcType c; double uC; // The center of the distribution in srcRange and unit range
        double  s,        uS; // The standard deviation of the distribution in srcRange and unit range
        double  g,        uG; // 1/(Sqrt(2) s) in srcRange and unit range
        
        double ErfA, ErfB, ErfAB;
        double K;     // The aspect ratio
        double m, uM; // \delta in the writeup
        double sDelta , dDelta; // quantum steps in the src and destination.
        
        double uLambda1, uLambda2;
        srcType lambda1,  lambda2;
        
        double uOmega1, uOmega2;
        srcType omega1,  omega2;
        
        double uOmegaP1, uOmegaP2;
        srcType omegaP1,  omegaP2;
        
        double TolDiscard = 1.0/16.0, TolKeep = 1.0/16.0, TolDistribute = 1.0/16.0;
        bool Qdiscard, Qdistribute, Qkeep;
        
        // dis(x) = disScale * erf( g * (x - c) ) + disConstant;
        dstType disConstant;
        double disScale;
        
        dstType disMin;              // The minimum value taken by the distribution/
        dstType linearConstant;      // The value added in the linear section of the distribution pDis(x) = x + linearConstant
        dstType shiftednErfConstant; // The height lost by using the linear distribution pDis(x) = dis(x) + shiftednErfConstant
        dstType disMax;              // The maximun value taken by the distribution.
        
        bool useLookUpTable;
                
        distributeErfParameters();
        
        distributeErfParameters(double sg, CV_32F_TYPE _uC,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_64F_TYPE _uC,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_8U_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_8S_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_16U_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_16S_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_32U_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_32S_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_64U_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        distributeErfParameters(double sg, CV_64S_TYPE _c,\
                                srcType sMin = srcInfo::min, srcType sMax = srcInfo::max, \
                                dstType dMin = dstInfo::min, dstType dMax = dstInfo::max
                                );
        
        void set(double, CV_32F_TYPE);
        void set(double, CV_64F_TYPE);
        void set(double, CV_8U_TYPE);
        void set(double, CV_8S_TYPE);
        void set(double, CV_16U_TYPE);
        void set(double, CV_16S_TYPE);
        void set(double, CV_32U_TYPE);
        void set(double, CV_32S_TYPE);
        void set(double, CV_64U_TYPE);
        void set(double, CV_64S_TYPE);
        
        void setCenter(CV_32F_TYPE);
        void setCenter(CV_64F_TYPE);
        void setCenter(CV_8U_TYPE);
        void setCenter(CV_8S_TYPE);
        void setCenter(CV_16U_TYPE);
        void setCenter(CV_16S_TYPE);
        void setCenter(CV_32U_TYPE);
        void setCenter(CV_32S_TYPE);
        void setCenter(CV_64U_TYPE);
        void setCenter(CV_64S_TYPE);
        
        void setUnitCenter(double);
        void setSrcCenter(srcType);
        
        
        void setUnitSigma(double _uS);
        void setSrcSigma(double _s);
        void setUnitG(double _uG);
        void setSrcG(double _g);
        
        void setInternals();
        
        void setDiscardRegionBounds();
        
        void setKeepRegionBounds();
        
        void setRegionQ();
        
        void setDistributionParameters();
        
        void setup();

    };
    
    
    template<int src_t, int dst_t>  class CV_EXPORTS depthConverter
    {
        public :
        virtual ~depthConverter<src_t, dst_t>(){};
        using srcInfo = cv_Data_Type<src_t>;
        using dstInfo = cv_Data_Type<dst_t>;
        using srcType = typename cv_Data_Type<src_t>::type;
        using dstType = typename cv_Data_Type<dst_t>::type;
        using wrkInfo = cv_Work_Type<src_t, dst_t>;
        using wrkType = typename cv_Work_Type<src_t, dst_t>::type;
        virtual void operator()(const srcType src, dstType &dst) = 0;
    };
    
    template<int src_t, int dst_t>  class  CV_EXPORTS distributeErf: public depthConverter<src_t, dst_t>
    {
        public :
        using srcType = typename distributeErf::srcType;
        using dstType = typename distributeErf::dstType;
        using wrkType = typename distributeErf::wrkType;
        distributeErfParameters<src_t, dst_t> par;
        dstType* map; // [cv::Data_Type<src_t>::max-cv::Data_Type<src_t>::min];
        
        distributeErf();
        distributeErf( distributeErfParameters<src_t, dst_t> par);
        distributeErf( double _g, srcType _c, srcType sMin, srcType sMax, dstType dMin, dstType dMax);
        void operator()(const srcType src, dstType &dst);
    };
    
    template<int src_t, int dst_t>  class CV_EXPORTS distributeErfCompact: public depthConverter<src_t, dst_t>
    {
        public :
        using srcType = typename distributeErfCompact::srcType;
        using dstType = typename distributeErfCompact::dstType;
        using wrkType = typename distributeErfCompact::wrkType;
        distributeErfParameters<src_t, dst_t> par;
        dstType* map; // [cv::Data_Type<src_t>::max-cv::Data_Type<src_t>::min];
        distributeErfCompact();
        distributeErfCompact( distributeErfParameters<src_t, dst_t> par);
        distributeErfCompact( double _g, srcType _c, srcType sMin, srcType sMax, dstType dMin, dstType dMax);
        void operator()(const srcType src, dstType &dst);
    };
    
    template<int src_t, int dst_t>  class CV_EXPORTS distributePartition: public depthConverter<src_t, dst_t>
    {
        public :
        using srcType = typename distributePartition::srcType;
        using dstType = typename distributePartition::dstType;
        using wrkType = typename distributePartition::wrkType;
        srcType sMinCutoff, sMaxCutoff;
        
        distributePartition();
        distributePartition( srcType sMinCutoff, srcType sMaxCutoff, srcType sMin, srcType sMax, dstType dMin, dstType dMax);
        void operator()(const srcType src, dstType &dst);
    };
    
    template<int src_t, int dst_t>  class CV_EXPORTS distributeLinear: public depthConverter<src_t, dst_t>
    {
        public :
        using srcType = typename distributeLinear::srcType;
        using dstType = typename distributeLinear::dstType;
        using wrkType = typename distributeLinear::wrkType;
        distributeErfParameters<src_t, dst_t> par;
        srcType fMin, fMax;
        wrkType g;
        dstType c, dMin, dMax;
        
        distributeLinear();
        distributeLinear( distributeErfParameters<src_t, dst_t> par);
        distributeLinear( double _g, double _c, srcType sMin, srcType sMax, dstType dMin, dstType dMax);
        void operator()(const srcType src, dstType dst) const;
    };

    
template<int src_t, int dst_t> class CV_EXPORTS colorSpaceConverter
{
    public :
    virtual ~colorSpaceConverter<src_t, dst_t>(){};
    using srcInfo = cv::Data_Type<src_t>;
    using srcType = typename cv::Data_Type<src_t>::type;
    using src_channel_type = srcType;
    
    using dstInfo = cv::Data_Type<dst_t>;
    using dstType = typename cv::Data_Type<dst_t>::type;
    using dst_channel_type = dstType;
    
    using wrkInfo  =  cv::Work_Type<src_t, dst_t>;
    using wrkType  = typename cv::Work_Type<CV_MAT_DEPTH(src_t), CV_MAT_DEPTH(dst_t)>::type;
    using sWrkInfo =  cv::Signed_Work_Type<src_t, dst_t>;
    using sWrkType = typename cv::Signed_Work_Type<CV_MAT_DEPTH(src_t), CV_MAT_DEPTH(dst_t)>::type;
    virtual void CV_EXPORTS operator()(const srcType * src, dstType* dst, int n) const = 0;
};

// template class colorSpaceConverter<CV_8UC3,CV_8UC3>;
// template class colorSpaceConverter<CV_8UC4,CV_8UC3>;
    
template<int src_t, int dst_t> class CV_EXPORTS RGB2Rot_int: public colorSpaceConverter<src_t, dst_t>
{
    public :
    using srcInfo  = typename RGB2Rot_int::srcInfo;
    using dstInfo  = typename RGB2Rot_int::dstInfo;
    using srcType  = typename RGB2Rot_int::srcType;
    using dstType  = typename RGB2Rot_int::dstType;
    using wrkInfo  = typename RGB2Rot_int::wrkInfo;
    using wrkType  = typename RGB2Rot_int::wrkType;
    using sWrkInfo = typename RGB2Rot_int::sWrkInfo;
    using sWrkType = typename RGB2Rot_int::sWrkType;
    // Values are expressed in three spaces; src, wrk, dst or source, rotated(working) and destination.
    // Values are expressed in two ranges; u, q or unitary and quantized
    
    int indxA{0}, indxB{1}, indxC{2}; // indices for the axis.
    int dstRGBIndices[3]; // indices for the destination 'RGB' channels
    int srcRGBIndices[3]; // indices for the source RGB channels
    Vec<wrkType, 3> qC_wrk; // The center point for the distribution function in the rotated color space
    Vec<double, 3>  uC_wrk; // The center point for the distribution function in the rotated color space scaled to 0:1
    Vec<srcType, 3> qC_src; // The center point for the distribution function in the source color space
    Vec<double, 3>  uC_src; // The center point for the distribution function in the source color space scaled to 0:1
    
    Vec<double, 3> uG; // The distribution parameter in the rotated color space scaled to 0:1
    
    Vec<double, 3> rRScale, nRScale, fRScale;
    cv::Matx<double, 3, 3> rR;
    
    cv::Matx<sWrkType, 3, 3> fR;
    Vec<sWrkType, 3> RRange, RMin, RMax;
    
    sWrkType qfR[3][3];
    double fScale[3], scale[3];
    
    Vec<double, dstInfo::channels> uRRange, uRMin, uRMax; // The range info for the result of the transformed space. The axis lengths and positions in the 0:1 space.
    
    distributeErfParameters<wrkInfo::dataType, dstInfo::dataType> redParam, greenParam, blueParam;
    depthConverter<wrkInfo::dataType, dstInfo::dataType> *redScale, *greenScale, *blueScale;
    
    RGB2Rot_int(); // todo set default distribution functions.
    
    RGB2Rot_int(const int srcBlueIdx, const int dstBlueIdx, const double theta, std::vector<double>  newG, std::vector<double> newC);
    RGB2Rot_int(const int srcBlueIdx, const int dstBlueIdx, const double theta, cv::Vec<double, 3> _g, cv::Vec<double, 3> _c);
    RGB2Rot_int(const int srcBlueIdx, const int dstBlueIdx, const double theta, double* g, double* c);
    
    Vec<typename cv::Signed_Work_Type<src_t, dst_t>::type, 3> toWrk(Vec<double, 3> pnt);
    Vec<typename cv::Data_Type<src_t>::type, 3>  toSrc(Vec<double, 3> pnt);
    Vec<typename cv::Data_Type<dst_t>::type, 3>  toDst(Vec<double, 3> pnt);
    Vec<double, 3> fromRot(Vec<double, 3> pnt);
    Vec<double, 3>   toRot(Vec<double, 3> pnt);
    void init();
    void setRGBIndices(int srcBlueIdx, int dstBlueIdx);
    void setTransformFromAngle(double theta );
    void setRanges();
    void setuCinSrc(Vec<double, 3> _c); // void setUnitC(Vec<double, 3> _c);
    void setuC(Vec<double, 3> _c); // void setUnitC(Vec<double, 3> _c);
    void setG(Vec<double, 3> _g);
    void setuG(Vec<double, 3> _g);
    void setRedDistributionErf();
    void setRedDistributionErf(  int center, double gradient);
    void setGreenDistributionErf();
    void setGreenDistributionErf(int center, double gradient);
    void setBlueDistributionErf();
    void setBlueDistributionErf( int center, double gradient);
    
    void operator()(const typename cv::Data_Type<src_t>::type* src, typename cv::Data_Type<dst_t>::type* dst, int n) const;
};


template<int src_t, int dst_t> class CV_EXPORTS RGB2Rot: public colorSpaceConverter<src_t, dst_t>
{
    public :
    using srcInfo = typename RGB2Rot::srcInfo;
    using dstInfo = typename RGB2Rot::dstInfo;
    using srcType = typename RGB2Rot::srcType;
    using dstType = typename RGB2Rot::dstType;
    using wrkInfo = typename RGB2Rot::wrkInfo;
    using wrkType = typename RGB2Rot::wrkType;
    using sWrkInfo = typename RGB2Rot::sWrkInfo;
    using sWrkType = typename RGB2Rot::sWrkType;
    // Values are expressed in three spaces; src, wrk, dst or source, rotated(working) and destination.
    // Values are expressed in two ranges; u, q or unitary and quantized
    
    int indxA{0}, indxB{1}, indxC{2}; // indices for the axis.
    int dstRGBIndices[3]; // indices for the destination 'RGB' channels
    int srcRGBIndices[3]; // indices for the source RGB channels
    wrkType C[3]; // The center point for the distribution function in the rotated color space
    Vec<wrkType, 3> qC_wrk; // The center point for the distribution function in the rotated color space
    Vec<double, 3>  uC_wrk; // The center point for the distribution function in the rotated color space scaled to 0:1
    Vec<srcType, 3> qC_src; // The center point for the distribution function in the source color space
    Vec<double, 3>  uC_src; // The center point for the distribution function in the source color space scaled to 0:1
    
    double qG[3];  // The distribution parameter in the rotated color space
    double uG[3]; // The distribution parameter in the rotated color space scaled to 0:1
    
    cv::Matx<double, 3, 3> uT, uiT; // The transformation matrix in 0:1 scale
    cv::Matx<double, 3, 3> R, fR; // The transformation matrix in 0:1 scale
    Vec<double, 3> fRScale; // The transformation matrix in 0:1 scale
    Vec<double, dstInfo::channels> uTRange, uTMin, uTMax; // The range info for the result of the transformed space. The axis lengths and positions in the 0:1 space.
    cv::Matx<sWrkType, 3, 3> T, iT; // The transformation matrix without overflow protection in the source / destination scale
    sWrkType M[dstInfo::channels][srcInfo::channels]; // The working matrix for the transform computation with overflow protection.
    sWrkType TRange[dstInfo::channels], TMin[dstInfo::channels], TMax[dstInfo::channels];
    
    depthConverter<wrkInfo::dataType, dstInfo::dataType> *redScale, *greenScale, *blueScale;
    
    RGB2Rot(); // todo set default distribution functions.
    
    RGB2Rot(const int srcBlueIdx, const int dstBlueIdx, const double theta, std::vector<double>  newG, std::vector<double> newC);
    RGB2Rot(const int srcBlueIdx, const int dstBlueIdx, cv::Matx<int, 3, 3>& T, cv::Vec<double, 3> _g, cv::Vec<int, 3> _c);
    
    RGB2Rot(const int srcBlueIdx, const int dstBlueIdx, const double theta, cv::Vec<double, 3> _g, cv::Vec<double, 3> _c);
    RGB2Rot(const int srcBlueIdx, const int dstBlueIdx, const double theta, double* g, double* c);
    
    void init();
    void setRGBIndices(int srcBlueIdx, int dstBlueIdx);
    void setTransform(cv::Matx<int, 3, 3>& T);
    void setTransformFromAngle(double theta );
    void setRanges();
    void setCinSrc(Vec<int, 3> _c);
    void setC(Vec<int, 3> _c);
    void setuCinSrc(Vec<double, 3> _c); // void setUnitC(Vec<double, 3> _c);
    void setuC(Vec<double, 3> _c); // void setUnitC(Vec<double, 3> _c);
    void setG(Vec<double, 3> _g);
    void setuG(Vec<double, 3> _g);
    void setRedDistributionErf();
    void setRedDistributionErf(  int center, double gradient);
    void setGreenDistributionErf();
    void setGreenDistributionErf(int center, double gradient);
    void setBlueDistributionErf();
    void setBlueDistributionErf( int center, double gradient);
    
    void operator()(const typename cv::Data_Type<src_t>::type* src, typename cv::Data_Type<dst_t>::type* dst, int n) const;
};

template<int src_t, int dst_t> void CV_EXPORTS convertColor(InputArray _src, OutputArray _dst, colorSpaceConverter<src_t, dst_t>& colorConverter);

template <typename Cvt> CV_EXPORTS_W void CvtColorLoop(const Mat& src, Mat& dst, const Cvt& cvt);
    
    

/*!
 The Base Class for 1D or Row-wise Filters

 This is the base class for linear or non-linear filters that process 1D data.
 In particular, such filters are used for the "horizontal" filtering parts in separable filters.

 Several functions in OpenCV return Ptr<BaseRowFilter> for the specific types of filters,
 and those pointers can be used directly or within cv::FilterEngine.
*/
class BaseRowFilter
{
public:
    //! the default constructor
    BaseRowFilter();
    //! the destructor
    virtual ~BaseRowFilter();
    //! the filtering operator. Must be overridden in the derived classes. The horizontal border interpolation is done outside of the class.
    virtual void operator()(const uchar* src, uchar* dst, int width, int cn) = 0;

    int ksize;
    int anchor;
};


/*!
 The Base Class for Column-wise Filters

 This is the base class for linear or non-linear filters that process columns of 2D arrays.
 Such filters are used for the "vertical" filtering parts in separable filters.

 Several functions in OpenCV return Ptr<BaseColumnFilter> for the specific types of filters,
 and those pointers can be used directly or within cv::FilterEngine.

 Unlike cv::BaseRowFilter, cv::BaseColumnFilter may have some context information,
 i.e. box filter keeps the sliding sum of elements. To reset the state BaseColumnFilter::reset()
 must be called (e.g. the method is called by cv::FilterEngine)
 */
class BaseColumnFilter
{
public:
    //! the default constructor
    BaseColumnFilter();
    //! the destructor
    virtual ~BaseColumnFilter();
    //! the filtering operator. Must be overridden in the derived classes. The vertical border interpolation is done outside of the class.
    virtual void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width) = 0;
    //! resets the internal buffers, if any
    virtual void reset();

    int ksize;
    int anchor;
};


/*!
 The Base Class for Non-Separable 2D Filters.

 This is the base class for linear or non-linear 2D filters.

 Several functions in OpenCV return Ptr<BaseFilter> for the specific types of filters,
 and those pointers can be used directly or within cv::FilterEngine.

 Similar to cv::BaseColumnFilter, the class may have some context information,
 that should be reset using BaseFilter::reset() method before processing the new array.
*/
class BaseFilter
{
public:
    //! the default constructor
    BaseFilter();
    //! the destructor
    virtual ~BaseFilter();
    //! the filtering operator. The horizontal and the vertical border interpolation is done outside of the class.
    virtual void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn) = 0;
    //! resets the internal buffers, if any
    virtual void reset();

    Size ksize;
    Point anchor;
};


/*!
 The Main Class for Image Filtering.

 The class can be used to apply an arbitrary filtering operation to an image.
 It contains all the necessary intermediate buffers, it computes extrapolated values
 of the "virtual" pixels outside of the image etc.
 Pointers to the initialized cv::FilterEngine instances
 are returned by various OpenCV functions, such as cv::createSeparableLinearFilter(),
 cv::createLinearFilter(), cv::createGaussianFilter(), cv::createDerivFilter(),
 cv::createBoxFilter() and cv::createMorphologyFilter().

 Using the class you can process large images by parts and build complex pipelines
 that include filtering as some of the stages. If all you need is to apply some pre-defined
 filtering operation, you may use cv::filter2D(), cv::erode(), cv::dilate() etc.
 functions that create FilterEngine internally.

 Here is the example on how to use the class to implement Laplacian operator, which is the sum of
 second-order derivatives. More complex variant for different types is implemented in cv::Laplacian().

 \code
 void laplace_f(const Mat& src, Mat& dst)
 {
     CV_Assert( src.type() == CV_32F );
     // make sure the destination array has the proper size and type
     dst.create(src.size(), src.type());

     // get the derivative and smooth kernels for d2I/dx2.
     // for d2I/dy2 we could use the same kernels, just swapped
     Mat kd, ks;
     getSobelKernels( kd, ks, 2, 0, ksize, false, ktype );

     // let's process 10 source rows at once
     int DELTA = std::min(10, src.rows);
     Ptr<FilterEngine> Fxx = createSeparableLinearFilter(src.type(),
     dst.type(), kd, ks, Point(-1,-1), 0, borderType, borderType, Scalar() );
     Ptr<FilterEngine> Fyy = createSeparableLinearFilter(src.type(),
     dst.type(), ks, kd, Point(-1,-1), 0, borderType, borderType, Scalar() );

     int y = Fxx->start(src), dsty = 0, dy = 0;
     Fyy->start(src);
     const uchar* sptr = src.data + y*src.step;

     // allocate the buffers for the spatial image derivatives;
     // the buffers need to have more than DELTA rows, because at the
     // last iteration the output may take max(kd.rows-1,ks.rows-1)
     // rows more than the input.
     Mat Ixx( DELTA + kd.rows - 1, src.cols, dst.type() );
     Mat Iyy( DELTA + kd.rows - 1, src.cols, dst.type() );

     // inside the loop we always pass DELTA rows to the filter
     // (note that the "proceed" method takes care of possibe overflow, since
     // it was given the actual image height in the "start" method)
     // on output we can get:
     //  * < DELTA rows (the initial buffer accumulation stage)
     //  * = DELTA rows (settled state in the middle)
     //  * > DELTA rows (then the input image is over, but we generate
     //                  "virtual" rows using the border mode and filter them)
     // this variable number of output rows is dy.
     // dsty is the current output row.
     // sptr is the pointer to the first input row in the portion to process
     for( ; dsty < dst.rows; sptr += DELTA*src.step, dsty += dy )
     {
         Fxx->proceed( sptr, (int)src.step, DELTA, Ixx.data, (int)Ixx.step );
         dy = Fyy->proceed( sptr, (int)src.step, DELTA, d2y.data, (int)Iyy.step );
         if( dy > 0 )
         {
             Mat dstripe = dst.rowRange(dsty, dsty + dy);
             add(Ixx.rowRange(0, dy), Iyy.rowRange(0, dy), dstripe);
         }
     }
 }
 \endcode
*/
class FilterEngine
{
public:
    //! the default constructor
    FilterEngine();
    //! the full constructor. Either _filter2D or both _rowFilter and _columnFilter must be non-empty.
    FilterEngine(const Ptr<BaseFilter>& _filter2D,
                 const Ptr<BaseRowFilter>& _rowFilter,
                 const Ptr<BaseColumnFilter>& _columnFilter,
                 int srcType, int dstType, int bufType,
                 int _rowBorderType = BORDER_REPLICATE,
                 int _columnBorderType = -1,
                 const Scalar& _borderValue = Scalar());
    //! the destructor
    virtual ~FilterEngine();
    //! reinitializes the engine. The previously assigned filters are released.
    void init(const Ptr<BaseFilter>& _filter2D,
              const Ptr<BaseRowFilter>& _rowFilter,
              const Ptr<BaseColumnFilter>& _columnFilter,
              int srcType, int dstType, int bufType,
              int _rowBorderType = BORDER_REPLICATE,
              int _columnBorderType = -1,
              const Scalar& _borderValue = Scalar());
    //! starts filtering of the specified ROI of an image of size wholeSize.
    virtual int start(Size wholeSize, Rect roi, int maxBufRows = -1);
    //! starts filtering of the specified ROI of the specified image.
    virtual int start(const Mat& src, const Rect& srcRoi = Rect(0,0,-1,-1),
                      bool isolated = false, int maxBufRows = -1);
    //! processes the next srcCount rows of the image.
    virtual int proceed(const uchar* src, int srcStep, int srcCount,
                        uchar* dst, int dstStep);
    //! applies filter to the specified ROI of the image. if srcRoi=(0,0,-1,-1), the whole image is filtered.
    virtual void apply( const Mat& src, Mat& dst,
                        const Rect& srcRoi = Rect(0,0,-1,-1),
                        Point dstOfs = Point(0,0),
                        bool isolated = false);
    //! returns true if the filter is separable
    bool isSeparable() const { return !filter2D; }
    //! returns the number
    int remainingInputRows() const;
    int remainingOutputRows() const;

    int srcType;
    int dstType;
    int bufType;
    Size ksize;
    Point anchor;
    int maxWidth;
    Size wholeSize;
    Rect roi;
    int dx1;
    int dx2;
    int rowBorderType;
    int columnBorderType;
    std::vector<int> borderTab;
    int borderElemSize;
    std::vector<uchar> ringBuf;
    std::vector<uchar> srcRow;
    std::vector<uchar> constBorderValue;
    std::vector<uchar> constBorderRow;
    int bufStep;
    int startY;
    int startY0;
    int endY;
    int rowCount;
    int dstY;
    std::vector<uchar*> rows;

    Ptr<BaseFilter> filter2D;
    Ptr<BaseRowFilter> rowFilter;
    Ptr<BaseColumnFilter> columnFilter;
};


//! returns type (one of KERNEL_*) of 1D or 2D kernel specified by its coefficients.
int getKernelType(InputArray kernel, Point anchor);

//! returns the primitive row filter with the specified kernel
Ptr<BaseRowFilter> getLinearRowFilter(int srcType, int bufType,
                                            InputArray kernel, int anchor,
                                            int symmetryType);

//! returns the primitive column filter with the specified kernel
Ptr<BaseColumnFilter> getLinearColumnFilter(int bufType, int dstType,
                                            InputArray kernel, int anchor,
                                            int symmetryType, double delta = 0,
                                            int bits = 0);

//! returns 2D filter with the specified kernel
Ptr<BaseFilter> getLinearFilter(int srcType, int dstType,
                                           InputArray kernel,
                                           Point anchor = Point(-1,-1),
                                           double delta = 0, int bits = 0);

//! returns the separable linear filter engine
Ptr<FilterEngine> createSeparableLinearFilter(int srcType, int dstType,
                          InputArray rowKernel, InputArray columnKernel,
                          Point anchor = Point(-1,-1), double delta = 0,
                          int rowBorderType = BORDER_DEFAULT,
                          int columnBorderType = -1,
                          const Scalar& borderValue = Scalar());

//! returns the non-separable linear filter engine
Ptr<FilterEngine> createLinearFilter(int srcType, int dstType,
                 InputArray kernel, Point _anchor = Point(-1,-1),
                 double delta = 0, int rowBorderType = BORDER_DEFAULT,
                 int columnBorderType = -1, const Scalar& borderValue = Scalar());

//! returns the Gaussian filter engine
Ptr<FilterEngine> createGaussianFilter( int type, Size ksize,
                                    double sigma1, double sigma2 = 0,
                                    int borderType = BORDER_DEFAULT);

//! returns filter engine for the generalized Sobel operator
Ptr<FilterEngine> createDerivFilter( int srcType, int dstType,
                                        int dx, int dy, int ksize,
                                        int borderType = BORDER_DEFAULT );

//! returns horizontal 1D box filter
Ptr<BaseRowFilter> getRowSumFilter(int srcType, int sumType,
                                              int ksize, int anchor = -1);

//! returns vertical 1D box filter
Ptr<BaseColumnFilter> getColumnSumFilter( int sumType, int dstType,
                                                     int ksize, int anchor = -1,
                                                     double scale = 1);
//! returns box filter engine
Ptr<FilterEngine> createBoxFilter( int srcType, int dstType, Size ksize,
                                              Point anchor = Point(-1,-1),
                                              bool normalize = true,
                                              int borderType = BORDER_DEFAULT);


//! returns horizontal 1D morphological filter
Ptr<BaseRowFilter> getMorphologyRowFilter(int op, int type, int ksize, int anchor = -1);

//! returns vertical 1D morphological filter
Ptr<BaseColumnFilter> getMorphologyColumnFilter(int op, int type, int ksize, int anchor = -1);

//! returns 2D morphological filter
Ptr<BaseFilter> getMorphologyFilter(int op, int type, InputArray kernel,
                                               Point anchor = Point(-1,-1));

//! returns morphological filter engine. Only MORPH_ERODE and MORPH_DILATE are supported.
CV_EXPORTS Ptr<FilterEngine> createMorphologyFilter(int op, int type, InputArray kernel,
                                                    Point anchor = Point(-1,-1), int rowBorderType = BORDER_CONSTANT,
                                                    int columnBorderType = -1,
                                                    const Scalar& borderValue = morphologyDefaultBorderValue());

static inline Point normalizeAnchor( Point anchor, Size ksize )
{
   if( anchor.x == -1 )
       anchor.x = ksize.width/2;
   if( anchor.y == -1 )
       anchor.y = ksize.height/2;
   CV_Assert( anchor.inside(Rect(0, 0, ksize.width, ksize.height)) );
   return anchor;
}

void preprocess2DKernel( const Mat& kernel, std::vector<Point>& coords, std::vector<uchar>& coeffs );
void crossCorr( const Mat& src, const Mat& templ, Mat& dst,
               Size corrsize, int ctype,
               Point anchor=Point(0,0), double delta=0,
               int borderType=BORDER_REFLECT_101 );

}

#endif

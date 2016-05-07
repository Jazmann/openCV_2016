/********************************************************************************
*
*
*  This program is demonstration for ellipse fitting. Program finds
*  contours and approximate it by ellipses.
*
*  Trackbar specify threshold parametr.
*
*  White lines is contours. Red lines is fitting ellipses.
*
*
*  Autor:  Denis Burenkov.
*
*
*
********************************************************************************/
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <chrono>
#include <random>

std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)

using namespace cv;
using namespace std;

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
        (clock_::now() - beg_).count(); }
    
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

static void help()
{
    cout <<
        "\nThis program is demonstration for ellipse fitting. The program finds\n"
        "contours and approximate it by ellipses.\n"
        "Call:\n"
        "./fitellipse [image_name -- Default ../data/stuff.jpg]\n" << endl;
}

int sliderPos = 56;

Mat image;

constexpr  char cv_Data_Type<CV_8U>::fmt[5];
constexpr  char cv_Data_Type<CV_8S>::fmt[5];
constexpr  char cv_Data_Type<CV_16U>::fmt[5];
constexpr  char cv_Data_Type<CV_16S>::fmt[5];
constexpr  char cv_Data_Type<CV_32U>::fmt[5];
constexpr  char cv_Data_Type<CV_32S>::fmt[5];
constexpr  char cv_Data_Type<CV_64U>::fmt[5];
constexpr  char cv_Data_Type<CV_64S>::fmt[5];
constexpr  char cv_Data_Type<CV_32F>::fmt[5];
constexpr  char cv_Data_Type<CV_64F>::fmt[5];


template<int dataType> void printImg( cv::InputArray _mat, const char* name){
    Mat mat = _mat.getMat();
    CV_Assert(dataType == mat.type());
    using typeInfo = cv::Data_Type<dataType>;
    using type = typename cv::Data_Type<dataType>::type;
    const int rows = mat.rows, cols = mat.cols;
    int channels = mat.channels();
    
    char fmt[10];
    if(CV_MAT_DEPTH(dataType) == CV_64F || CV_MAT_DEPTH(dataType) == CV_32F){
        sprintf(fmt,"%s","%14.12f");
    }else{
        strncpy(fmt, typeInfo::fmt, sizeof(fmt));
    }
    
    fprintf (stdout,"%s",name);
    fprintf (stdout, " ={\n");
    switch(channels)
    {
        case 1:
        {
            MatIterator_<type> it, end;
            int i=0;
            for( it = mat.begin<type>(), end = mat.end<type>(); it != end; ++it)
            {
                int col = i % cols;
                int row = floor(i/cols);
                if(col==0){fprintf (stdout, "{");}
                fprintf (stdout, fmt, (*it));
                if(col<cols-1){
                    fprintf(stdout, ", ");
                }else{
                    fprintf(stdout, "}\n ");
                    if(row<rows-1){
                        fprintf(stdout, ",\n");
                        
                    }
                }
                i++; 
            }
            break;
        }
        case 3:
        {
            MatIterator_<Vec<type,3>> it, end;
            int i=0;
            for( it = mat.begin<Vec<type,3>>(), end = mat.end<Vec<type,3>>(); it != end; ++it)
            {
                int col = i % cols;
                int row = floor(i/cols);
                if(col==0){fprintf (stdout, "{");}
                fprintf (stdout, "{");
                fprintf (stdout, fmt, (*it)[0]); fprintf(stdout, ", ");
                fprintf (stdout, fmt, (*it)[1]); fprintf(stdout, ", ");
                fprintf (stdout, fmt, (*it)[2]); fprintf(stdout, "} ");
                if(col<cols-1){
                    fprintf(stdout, ", ");
                }else{
                    fprintf(stdout, "}\n ");
                    if(row<rows-1){
                        fprintf(stdout, ",\n");

                    }
                }
                i++;
            }
            break;
        }
        case 4:
        {
            MatIterator_<Vec<type,4>> it, end;
            int i=0;
            for( it = mat.begin<Vec<type,4>>(), end = mat.end<Vec<type,4>>(); it != end; ++it)
            {
                int col = i % cols;
                int row = floor(i/cols);
                if(col==0){fprintf (stdout, "{");}
                fprintf (stdout, "{");
                fprintf (stdout, fmt, (*it)[0]); fprintf(stdout, ", ");
                fprintf (stdout, fmt, (*it)[1]); fprintf(stdout, ", ");
                fprintf (stdout, fmt, (*it)[2]); fprintf(stdout, ", ");
                fprintf (stdout, fmt, (*it)[3]); fprintf(stdout, "} ");
                if(col<cols-1){
                    fprintf(stdout, ", ");
                }else{
                    fprintf(stdout, "}\n ");
                    if(row<rows-1){
                        fprintf(stdout, ",\n");
                        
                    }
                }
                i++;
            }
        }

    }
    fprintf (stdout, "};\n");

    }


//template<int dataType> void printImg( cv::InputArray _mat, const char* name){
//    Mat mat = _mat.getMat();
//    CV_Assert(dataType == mat.type());
//    using typeInfo = cv::Data_Type<dataType>;
//    const int rows = mat.rows, cols = mat.cols;
//    const int channels = mat.channels();
//    char fmt[5];
//    strncpy(fmt, cv::Data_Type<dataType>::fmt, sizeof(fmt));
//    
//    fprintf (stdout,"%s",name);
//    fprintf (stdout, " ={\n");
//    for (int r=0; r < rows; r++) {
//        fprintf (stdout, "{\n");
//        for (int c=0; c < cols; c++) {
//            Vec< typename typeInfo::type, typeInfo::channels> RGBColor = mat.at<Vec< typename typeInfo::type, typeInfo::channels>>(Point(r,c));
//            if (channels > 1) {
//                fprintf (stdout, "{");
//                for (int i=1; i<channels; i++) {
//                    fprintf (stdout, typeInfo::fmt, RGBColor(i-1)); fprintf(stdout, ", ");
//                }
//                fprintf (stdout, typeInfo::fmt, RGBColor(channels-1)); fprintf(stdout,  "} ");
//            } else {
//                fprintf (stdout, typeInfo::fmt, RGBColor(0));
//            }
//            if (c < cols-1){fprintf (stdout, ",");};
//        }
//        fprintf (stdout, "\n }");
//        if (r < rows-1){fprintf (stdout, ",");}
//    }
//    fprintf (stdout, "};\n");
//}

template<int src_t,int dst_t> void printDist( depthConverter<src_t, dst_t> *dis, const char* name){
    
    using srcInfo = cv::Data_Type<src_t>;
    using srcType = typename cv::Data_Type<src_t>::type;
    
    using dstInfo = cv::Data_Type<dst_t>;
    using dstType = typename cv::Data_Type<dst_t>::type;
    const int maxPoints = 256;
    const int range = srcInfo::max-srcInfo::min+1;
    const int dstRange = dstInfo::max-dstInfo::min+1;
    const int step = std::ceil(double(range)/double(maxPoints));
    srcType src[range];
    dstType dst[dstRange];
    char buf[100];
    strcpy(buf, "{"); strcat(buf, srcInfo::fmt); strcat(buf, ", "); strcat(buf, dstInfo::fmt); strcat(buf, "}");
    fprintf (stdout, "%s",name);
    fprintf (stdout, " ={\n");
    for (int i=srcInfo::min; i <= srcInfo::max; i = i + step) {
        int indx = i-srcInfo::min;
        src[indx] = srcType(i);
        (*dis)(src[indx],dst[indx]);
        fprintf (stdout, buf,src[indx],dst[indx]);
        if (i<srcInfo::max-step+1){fprintf (stdout, ",");};
        if (i % 10 ==0){fprintf (stdout, "\n");};
    }
    fprintf (stdout, "};\n");
}

bool yesno(String question){
    char type;
    do
    {
        cout << question << " [y/n]" << endl;
        cin >> type;
    }
    while( !cin.fail() && type!='y' && type!='n' );
    if(type=='y'){
        return 1;
    }else{
        return 0;
    }
    }


void processImage(int, void*);

int main( int argc, char** argv )
{
    const int src_t = CV_8S, dst_t = CV_8U;
    
    using srcInfo = cv::Data_Type<src_t>;
    using srcType = typename cv::Data_Type<src_t>::type;
    
    using dstInfo = cv::Data_Type<dst_t>;
    using dstType = typename cv::Data_Type<dst_t>::type;
    // Assuming that any matrix rotation id performed with a matrix of the same bit depth as the srcType.
    using sWrkInfo =  cv::Signed_Work_Type<src_t, src_t>;
    using sWrkType = typename cv::Signed_Work_Type<CV_MAT_DEPTH(src_t), CV_MAT_DEPTH(src_t)>::type;
    
    // ****************************************************
    
    //  erf test
    if (yesno("Run erf Test?")) {
        fprintf (stdout, "erf ={\n");
        for( int j = -255; j < 255; j++ )
        {
            double x = double(j)/128.0;
            fprintf (stdout, "{ %f, %f }",x,cv::erf(x));
            if (j<255-1){fprintf (stdout, ",");};
            if (j % 10 ==0){fprintf (stdout, "\n");};
        }
        fprintf (stdout, "};\n");
    }
    // dqAS Test
    
    if (yesno("Run dqAS Test?")) {
        cv::Vec<int, 3> dqASout;
        dqASout = cv::dqAS( 0.1);
        fprintf (stdout, "dqAS(%f) ={%d, %d, %d }\n", 0.1, dqASout(0), dqASout(1), dqASout(2));
    }
    
    
    // distributeErfParameters Test
    if (yesno("Run distributeErfParameters Test?")) {
       cv::distributeErfParameters<src_t, dst_t> par(0.05,0.65);
       par.print();
    }
    
    // Dist Test
    if (yesno("Run Dist Test?")) {
        cv::distributeErfParameters<src_t, dst_t> par(0.05,0.65);
        cv::distributeErf<src_t, dst_t> pDisErf(par);
        cv::distributeErfCompact<src_t, dst_t> pDisErfCompact(par);
        cv::distributeLinear<src_t, dst_t> pDisLinear(par);
        cv::distributeStep<src_t, dst_t> pDisStep(par);
        cv::distributePartition<src_t, dst_t> pDisPartition(par);
        depthConverter<src_t, dst_t> *pDis;
        pDis = distribute<src_t, dst_t>(par);
        
        //  pDis test
        printDist<src_t,dst_t>(pDis,"pDis");
        printDist<src_t,dst_t>(&pDisErf,"pDisErf");
        printDist<src_t,dst_t>(&pDisErfCompact,"pDisErfCompact");
        printDist<src_t,dst_t>(&pDisLinear,"pDisLinear");
        printDist<src_t,dst_t>(&pDisStep,"pDisStep");
        printDist<src_t,dst_t>(&pDisPartition,"pDisPartition");
    }
    
    // Rotation with the actual values.
    if (yesno("Run Rotation with the actual values Test?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{-0.235702, -27.0915, -61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};
        
        RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
        fprintf(stdout,"(* %s\n *)","Luminocity");
        rot.LParam.print();
        printDist<sWrkInfo::dataType,dst_t>(rot.LDist, "rotLDist");
        fprintf(stdout,"(* %s\n *)","Chromatic A");
        rot.CaParam.print();
        printDist<sWrkInfo::dataType,dst_t>(rot.CaDist,"rotCaDist");
        fprintf(stdout,"(* %s\n *)","Chromatic B");
        rot.CbParam.print();
        printDist<sWrkInfo::dataType,dst_t>(rot.CbDist,"rotCbDist");
        
        // create a new 256x256 image
        Mat rgbImg(Size(256,256),CV_8UC3);
        Mat LCaCbImg(Size(256,256),CV_8UC3);
        
        for (int r=0; r <= 255; r++) {
            for (int g=0; g <= 255; g++) {
                char b=255-ceil((r+g)/2.0) ;
                Vec3b rgbColor{r,g,b};
                rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
            }
        }
        
        for (int r=0; r <= 255; r++) {
            for (int g=0; g <= 255; g++) {
                Vec3b LCaCbColor = rot.apply(rgbImg.at<Vec3b>(Point(r,g)));
                LCaCbImg.at<Vec3b>(Point(r,g)) = LCaCbColor;
            }
        }
        
        namedWindow("RGB Out",1);
        imshow("RGB Out", rgbImg);
        namedWindow("LCaCb Out",1);
        imshow("LCaCb Out", LCaCbImg);
        waitKey(); // Wait for a key stroke; the same function arranges events processing
    };
    
    // Check convertColor
    
    if (yesno("Run Check convertColor Test?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{-0.235702, -27.0915, -61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};

        RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
        // create a new 256x256 image
        Mat rgbImg(Size(256,256),CV_8UC3);
        for (int r=0; r <= 255; r++) {
            for (int g=0; g <= 255; g++) {
                char b=255-ceil((r+g)/2.0) ;
                Vec3b rgbColor{r,g,b};
                rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
            }
        }
        
        Mat LCaCbImg(Size(256,256),CV_8UC3);
        cv::convertColor<CV_8UC3,CV_8UC3>(rgbImg, LCaCbImg, rot);
        
        namedWindow("RGB Out",1);
        imshow("RGB Out", rgbImg);
        namedWindow("convertColor LCaCb Out",1);
        imshow("convertColor LCaCb Out", LCaCbImg);
        waitKey(); // Wait for a key stroke; the same function arranges events processing
    }
    
    // Check convertColor with classifier
    
    if (yesno("Run Check convertColor with classifier Test?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{-0.235702, -27.0915, -61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};

        RGB2Rot<CV_8UC3,CV_8UC4> rot(2, 2, theta, uG, uC);
        // create a new 256x256 image
        Mat rgbImg(Size(256,256),CV_8UC3);
        for (int r=0; r <= 255; r++) {
            for (int g=0; g <= 255; g++) {
                char b=255-ceil((r+g)/2.0) ;
                Vec3b rgbColor{r,g,b};
                rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
            }
        }
        
        Mat LCaCbImg(Size(256,256),CV_8UC4);
        cv::convertColor<CV_8UC3,CV_8UC4>(rgbImg, LCaCbImg, rot);
        
        printImg<CV_8UC4>(LCaCbImg,"LCaCbImg");
                

        Point end(0,0), start(109,181), vec(0,1);
        
        vec.x  = 0;  vec.y  = 1;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  = 0;  vec.y  =-1;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  = 1;  vec.y  = 0;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  =-1;  vec.y  = 0;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  = 1;  vec.y  = 1;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  = 1;  vec.y  =-1;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  =-1;  vec.y  = 1;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        vec.x  =-1;  vec.y  =-1;
        end = cv::runReach<CV_8UC4>(LCaCbImg, start, vec);
        fprintf (stdout, "end ={%d, %d }\n", end.x, end.y);
        
        // {{127, 163}, {88, 202}}
        
        start = Point(127, 163);
        end   = Point( 88, 202);
        
        Mat pnts = fillamentFill<CV_8UC4>(LCaCbImg, start, end);
        printImg<CV_32SC1>(pnts,"pnts");
        
        end = runReachToEnd<CV_8UC4>(LCaCbImg,  start, Point(-1,1));
        fprintf (stdout, "runReachToEnd ={{%d,%d},{%d, %d }}\n", start.x, start.y, end.x, end.y);
        
        Mat midPnts = runReachMidlineMat<CV_8UC4>(LCaCbImg,  start, Point(-1,0));
        printImg<CV_32SC1>(midPnts,"midPnts");
        
        cv::Mat3b roiMat = LCaCbImg(cv::Rect(88,163,127,202));
        cv::Scalar mean;
        mean =  cv::mean(roiMat);
        fprintf (stdout, "mean ={%f, %f, %f }\n", mean[0], mean[1], mean[2]);
        
        namedWindow("RGB Out",1);
        imshow("RGB Out", rgbImg);
        namedWindow("convertColor LCaCb Out",1);
        imshow("convertColor LCaCb Out", LCaCbImg);
        waitKey(); // Wait for a key stroke; the same function arranges events processing
    }

    // Check floating point method.
    if (yesno("Run Floating Point Method Test?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{-0.235702, -27.0915, -61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};

        RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
        // create a new 256x256 image
        Mat rgbImg(Size(256,256),CV_8UC3);
        for (int r=0; r < rgbImg.rows; r++) {
            for (int g=0; g < rgbImg.cols; g++) {
                char b=255-ceil((r+g)/2.0) ;
                Vec3b rgbColor{r,g,b};
                rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
            }
        }
        Mat LCaCbImg(Size(256,256),CV_8UC3);
        // Set the distribution region boundary constants.
        distributeErfParameters<sWrkInfo::dataType, CV_8U> LParam, CaParam, CbParam;
        LParam.init();
        LParam.set(-1.0*uG(0), uC(0));
        LParam.setRange(0, ceil(rot.L(0)*(srcInfo::max - srcInfo::min)), dstInfo::min, dstInfo::max);
        CaParam.init();
        CaParam.set(-1.0*uG(1), uC(1));
        CaParam.setRange(floor(-rot.L(1)*(srcInfo::max - srcInfo::min)/2.), ceil(rot.L(1)*(srcInfo::max - srcInfo::min)/2.),dstInfo::min,dstInfo::max);
        CbParam.init();
        CbParam.set(-1.0*uG(2), uC(2));
        CbParam.setRange(floor(-rot.L(2)*(srcInfo::max - srcInfo::min)/2.), ceil(rot.L(2)*(srcInfo::max - srcInfo::min)/2.),dstInfo::min,dstInfo::max);
        
        distributeErf<sWrkInfo::dataType, dst_t> LDistDouble(LParam);
        distributeErf<sWrkInfo::dataType, dst_t> CaDistDouble(CaParam);
        distributeErf<sWrkInfo::dataType, dst_t> CbDistDouble(CbParam);
        
        printDist<sWrkInfo::dataType,dst_t>(&LDistDouble, "LDistDouble");
        printDist<sWrkInfo::dataType,dst_t>(&CaDistDouble,"CaDistDouble");
        printDist<sWrkInfo::dataType,dst_t>(&CbDistDouble,"CbDistDouble");
        
        for (int r=0; r < rgbImg.rows; r++) {
            for (int c=0; c < rgbImg.cols; c++) {
                Matx<double,3,1> rgbColor = rgbImg.at<Vec3b>(Point(r,c));
                Matx<double,3,1> LCaCbColor = rot.rR * rgbColor;
                Vec<sWrkType,3> rotColor{sWrkType(rot.rRScale(0) * LCaCbColor(0)), sWrkType(rot.rRScale(1) * LCaCbColor(1)),sWrkType(rot.rRScale(2) * LCaCbColor(2))};
                Vec<dstType,3> skinColor;
                LDistDouble( rotColor(0),skinColor(0));
                CaDistDouble(rotColor(1),skinColor(1));
                CbDistDouble(rotColor(2),skinColor(2));
                LCaCbImg.at<Vec3b>(Point(r,c)) = skinColor;
            }
        }
        namedWindow("Floating Point Method Test RGB Out",1);
        imshow("Floating Point Method Test RGB Out", rgbImg);
        namedWindow("Floating Point Method Test LCaCb Out",1);
        imshow("Floating Point Method Test LCaCb Out", LCaCbImg);
        waitKey(); // Wait for a key stroke; the same function arranges events processing
    }
    
    
    if (yesno("Run KinkFit Test?")) {
    cv::Matx<double,100,2> testPnts{       45.,52.888151769357144,45.61290322580645,53.379903331559454,46.225806451612904,
        54.43678274022489,46.83870967741935,54.922353176186164,47.45161290322581,
        55.94608712858764,48.064516129032256,56.23149251282879,48.67741935483871,
        57.38190834373175,49.29032258064516,57.669695363790524,49.903225806451616,
        60.17619598286242,50.516129032258064,61.86290622136226,51.12903225806452,
        59.80082123341609,51.74193548387097,62.715220680976465,52.354838709677416,
        61.2047583433808,52.96774193548387,63.71869387397682,53.58064516129032,
        65.02897400999153,54.193548387096776,63.20778185088133,54.806451612903224,
        65.67579021559645,55.41935483870968,65.98369596561176,56.03225806451613,
        66.7934223776521,56.645161290322584,67.18596979794667,57.25806451612903,
        67.50313865530488,57.87096774193548,68.97549463038473,58.483870967741936,
        69.88077622651754,59.096774193548384,71.36694275620557,59.70967741935484,
        71.78022470387582,60.32258064516129,73.1591600461583,60.935483870967744,
        73.81373168287564,61.54838709677419,76.57495616965004,62.16129032258065,
        73.24802488761028,62.774193548387096,75.6656250507314,63.38709677419355,
        74.55393824113169,64.,77.2003709710424,64.,76.66593321719735,64.45714285714286,
        75.09496715271288,64.91428571428571,74.84944763060817,65.37142857142857,
        75.22687977776656,65.82857142857142,74.0152903292938,66.28571428571429,
        74.46358347987734,66.74285714285715,73.78397132480252,67.2,70.9839860866106,
        67.65714285714286,68.9712056900418,68.11428571428571,70.19994517250525,
        68.57142857142857,67.65705644038334,69.02857142857142,67.13493324965029,
        69.48571428571428,65.5582122076453,69.94285714285714,64.5919710350268,70.4,
        65.20668820112046,70.85714285714286,63.46193618944044,71.31428571428572,
        63.59842894963043,71.77142857142857,61.309743889146965,72.22857142857143,
        62.28764882297187,72.68571428571428,62.74392999233948,73.14285714285714,
        59.38607956556047,73.6,57.33807691673903,74.05714285714286,58.5583435321059,
        74.51428571428572,56.940438168213355,74.97142857142858,55.95656839088099,
        75.42857142857143,54.75969984635866,75.88571428571429,53.55314684605367,
        76.34285714285714,52.71132706259367,76.8,52.63845645319114,77.25714285714285,
        51.67844974308407,77.71428571428571,49.99557182108738,78.17142857142858,
        51.88716632105374,78.62857142857143,48.62720875473051,79.08571428571429,
        46.83919820517314,79.54285714285714,46.28321149488251,80.,47.161653372303256,
        80.45714285714286,44.586792903986066,80.91428571428571,44.03642517026495,
        81.37142857142857,44.39158557660236,81.82857142857142,43.198497341996706,
        82.28571428571429,42.82374225882003,82.74285714285715,39.05276784174136,83.2,
        42.73869103217426,83.65714285714286,39.8290221941061,84.11428571428571,
        37.8772279844991,84.57142857142857,37.87298080671891,85.02857142857142,
        38.46263874744129,85.48571428571428,36.257228483992655,85.94285714285714,
        34.59495327646449,86.4,34.67710431529364,86.85714285714286,33.899979627243006,
        87.31428571428572,33.05590724596813,87.77142857142857,31.743128365130417,
        88.22857142857143,31.104055224825142,88.68571428571428,26.59222363920442,
        89.14285714285714,29.863393754223978,89.6,28.24717728991598,90.05714285714286,
        27.39612367754152,90.51428571428572,26.787519398308135,90.97142857142858,
        25.673297086020813,91.42857142857143,24.14423551289683,91.88571428571429,
        22.66045461128192,92.34285714285714,22.64120283419664,92.8,22.088189457415133,
        93.25714285714285,22.394657553218533,93.71428571428571,19.387473487651725,
        94.17142857142858,20.073554437747884,94.62857142857143,20.490564076704622,
        95.08571428571429,17.560506050375892,95.54285714285714,16.432003139875423,96.,
        15.372283319509826};
    
        cv::Matx<double,3,2> line;
        kinkFitLine( testPnts,  line, CV_DIST_L2, 0, 0.01, 0.01 );
        
        fprintf (stdout, "line ={{%f, %f}, {%f, %f},{%f, %f}}\n", line(0,0), line(0,1), line(1,0), line(1,1), line(2,0), line(2,1));
}


    // Run Timings
    if (yesno("Run Timings?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{-0.235702, -27.0915, -61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};

        RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
        
        // Set the distribution region boundary constants.
        distributeErfParameters<sWrkInfo::dataType, CV_8U> LParam, CaParam, CbParam;
        LParam.init();
        LParam.set(-1.0*uG(0), uC(0));
        LParam.setRange(0, ceil(rot.L(0)*(srcInfo::max - srcInfo::min)), dstInfo::min, dstInfo::max);
        CaParam.init();
        CaParam.set(-1.0*uG(1), uC(1));
        CaParam.setRange(floor(-rot.L(1)*(srcInfo::max - srcInfo::min)/2.), ceil(rot.L(1)*(srcInfo::max - srcInfo::min)/2.),dstInfo::min,dstInfo::max);
        CbParam.init();
        CbParam.set(-1.0*uG(2), uC(2));
        CbParam.setRange(floor(-rot.L(2)*(srcInfo::max - srcInfo::min)/2.), ceil(rot.L(2)*(srcInfo::max - srcInfo::min)/2.),dstInfo::min,dstInfo::max);
        
        distributeErf<sWrkInfo::dataType, dst_t> LDistDouble(LParam);
        distributeErf<sWrkInfo::dataType, dst_t> CaDistDouble(CaParam);
        distributeErf<sWrkInfo::dataType, dst_t> CbDistDouble(CbParam);

        
        int rows=256, cols=256;
        // create a new 256x256 image
        Mat rgbImg(Size(rows,cols),CV_8UC3);
        for (int r=0; r < rgbImg.rows; r++) {
            for (int g=0; g < rgbImg.cols; g++) {
                char b=255-ceil((r+g)/2.0) ;
                Vec3b rgbColor{r,g,b};
                rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
            }
        }
        
        // Setup output array.
        Mat LCaCbImg(Size(rows,cols),CV_8UC3);
        
        Timer tmr;
        
        // Number of runs.
        const int runs=1000;
        const int goes=100;
      //  cv::Matx<double, runs, goes> floatTimes, intTimes;
        cv::Matx<double, goes, 1> floatTimes, intTimes;
        fprintf(stdout,"%s","runLms = {");
        for (int go=0; go < goes; go++) {
            std::uniform_int_distribution<int> randomMin(0,64), randomMax(192,255) ; // guaranteed unbiased
            std::uniform_int_distribution<int> red(randomMin(rng),randomMax(rng)), green(randomMin(rng),randomMax(rng)), blue(randomMin(rng),randomMax(rng));
            
            fprintf(stdout,"{{%d,%d},{%d,%d},{%d,%d}}",red.min(),red.max(),green.min(),green.max(),blue.min(),blue.max());
            if (go < goes){fprintf (stdout, ",");};
            if (go % 10 ==0){fprintf (stdout, "\n");};
            
            for (int r=0; r < rgbImg.rows; r++) {
                for (int g=0; g < rgbImg.cols; g++) {
                    Vec3b rgbColor{red(rng),green(rng),blue(rng)};
                    rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
                }
            }

            // Timed loop1
            floatTimes(go,0) = 100.;
            for (int run=0; run < runs; run++) {
                tmr.reset();
                for (int r=0; r < rgbImg.rows; r++) {
                    for (int c=0; c < rgbImg.cols; c++) {
                        Matx<double,3,1> rgbColor = rgbImg.at<Vec3b>(Point(r,c));
                        Matx<double,3,1> LCaCbColor = rot.rR * rgbColor;
                        Vec<sWrkType,3> rotColor{sWrkType(rot.rRScale(0) * LCaCbColor(0)), sWrkType(rot.rRScale(1) * LCaCbColor(1)),sWrkType(rot.rRScale(2) * LCaCbColor(2))};
                        Vec<dstType,3> skinColor;
                        LDistDouble( rotColor(0),skinColor(0));
                        CaDistDouble(rotColor(1),skinColor(1));
                        CbDistDouble(rotColor(2),skinColor(2));
                        LCaCbImg.at<Vec3b>(Point(r,c)) = skinColor;
                    }
                }
              //  floatTimes(go,run) = tmr.elapsed();
                if(floatTimes(go,0) > tmr.elapsed()){
                   floatTimes(go,0) = tmr.elapsed();
                }
                
            }
            
            // Timed loop2
            
            intTimes(go,0) = 100.;
            for (int run=0; run < runs; run++) {
                tmr.reset();
                cv::convertColor<CV_8UC3,CV_8UC3>(rgbImg, LCaCbImg, rot);
                //   intTimes(run,go) = tmr.elapsed();
                if(intTimes(go,0) > tmr.elapsed()){
                    intTimes(go,0) = tmr.elapsed();
                }

            }
            
        }
        fprintf(stdout,"%s","}\n");
        
        // print out
            
            printImg<CV_64FC1>(intTimes,"intTimes");
            
            printImg<CV_64FC1>(floatTimes,"floatTimes");
            
//        fprintf (stdout, "floatTimes ={\n");
//        for( int j = 0; j < runs; j++ )
//        {
//            fprintf (stdout, "%14.11f ",floatTimes(j));
//            if (j<runs-1){fprintf (stdout, ",");};
//            if (j % 10 ==0){fprintf (stdout, "\n");};
//        }
//        fprintf (stdout, "};\n");
//        
//        fprintf (stdout, "intTimes ={\n");
//        for( int j = 0; j < runs; j++ )
//        {
//            fprintf (stdout, "%14.11f ",intTimes(j));
//            if (j<runs-1){fprintf (stdout, ",");};
//            if (j % 10 ==0){fprintf (stdout, "\n");};
//        }
//        fprintf (stdout, "};\n");
    }

    
    Mat rgbImg(Size(256,256),CV_8UC3);
    for (int r=0; r < rgbImg.rows; r++) {
        for (int g=0; g < rgbImg.cols; g++) {
            char b=255-ceil((r+g)/2.0) ;
            Vec3b rgbColor{r,g,b};
            rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
        }
    }
    printImg<CV_8UC3>(rgbImg,"rgbImgTest");
    
    double theta = 0.902576829326826;
    cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
    cv::Vec<double, 3> uG{0.235702, 27.0915, 61.4471};
    cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};
    RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
    Mat LCaCbImg(Size(256,256),CV_8UC3);
    cv::convertColor<CV_8UC3,CV_8UC3>(rgbImg, LCaCbImg, rot);
    
    printImg<CV_8UC3>(LCaCbImg,"LCaCbImg");
    

    
    // ****************************************************
    
    cv::CommandLineParser parser(argc, argv,
        "{help h||}{@image|../data/stuff.jpg|}"
    );
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    string filename = parser.get<string>("@image");
    image = imread(filename, 0);
    if( image.empty() )
    {
        cout << "Couldn't open image " << filename << "\n";
        return 0;
    }

    imshow("source", image);
    namedWindow("result", 1);

    // Create toolbars. HighGUI use.
    createTrackbar( "threshold", "result", &sliderPos, 255, processImage );
    processImage(0, 0);

    // Wait for a key stroke; the same function arranges events processing
    waitKey();
    return 0;
}

// Define trackbar callback functon. This function find contours,
// draw it and approximate it by ellipses.
void processImage(int /*h*/, void*)
{
    vector<vector<Point> > contours;
    Mat bimage = image >= sliderPos;

    findContours(bimage, contours, RETR_LIST, CHAIN_APPROX_NONE);

    Mat cimage = Mat::zeros(bimage.size(), CV_8UC3);
    
    fprintf (stdout, "dataSets = {};\n");
    for(size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();
        if( count < 6 )
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
                RotatedRect box = fitEllipse(pointsf);
        RotatedRect boxAMS = fitEllipseAMS(pointsf);
        RotatedRect boxDirect = fitEllipseDirect(pointsf);
        

        if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*30 )
            continue;
        
        if (true) { // fabs(box.angle-boxDirect.angle)>3
            fprintf (stdout, "(*-------------------------------------------------------*)\n");
            fprintf (stdout, "(*contour %zu of %lu *)\n", i, contours.size());
            fprintf (stdout, "AppendTo[dataSets, %lu ];\n", i);
            fprintf (stdout, "(*Angle Wrong : %f - %f  = %f *)\n", box.angle, boxDirect.angle, box.angle-boxDirect.angle);
            const Point* ptsi = pointsf.ptr<Point>();
            const Point2f* ptsf = pointsf.ptr<Point2f>();
            fprintf (stdout, "points%zuCpp={\n",i);
            for( int j = 0; j < pointsf.checkVector(2); j++ )
            {
                Point2f p = (pointsf.depth() == CV_32F) ? ptsf[j] : Point2f((float)ptsi[j].x, (float)ptsi[j].y);
                fprintf (stdout, "{ %f, %f }",p.x, p.y);
                if (j<pointsf.checkVector(2)-1){fprintf (stdout, ",");};
                if (fmod(pointsf.checkVector(2),5.0)==0){fprintf (stdout, "\n");};
            }
            fprintf (stdout, "};\n");
            fprintf (stdout, "ellipse%zuCpp={ {%1.14e, %1.14e},{ %1.14e, %1.14e}, %1.14e };\n",i,box.center.x, box.center.y, box.size.width/2.0, box.size.height/2.0, box.angle);
            fprintf (stdout, "ellipseAMS%zuCpp={ {%1.14e, %1.14e},{ %1.14e, %1.14e}, %1.14e };\n",i,boxAMS.center.x, boxAMS.center.y, boxAMS.size.width/2.0, boxAMS.size.height/2.0, boxAMS.angle);
            fprintf (stdout, "ellipseDirect%zuCpp={ {%1.14e, %1.14e},{ %1.14e, %1.14e}, %1.14e };\n",i,boxDirect.center.x, boxDirect.center.y, boxDirect.size.width/2.0, boxDirect.size.height/2.0, boxDirect.angle);
            fprintf (stdout, "(*-------------------------------------------------------*)\n");
        }
        
        drawContours(cimage, contours, (int)i, Scalar::all(255), 1, 8);

        ellipse(cimage, box, Scalar(0,255,255), 1, LINE_AA);
        ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, LINE_AA);
        
        ellipse(cimage, boxAMS, Scalar(0,0,255), 1, LINE_AA);
        ellipse(cimage, boxAMS.center, boxAMS.size*0.5f, boxAMS.angle, 0, 360, Scalar(0,255,255), 1, LINE_AA);
        
        ellipse(cimage, boxDirect, Scalar(255,0,255), 1, LINE_AA);
        ellipse(cimage, boxDirect.center, boxDirect.size*0.5f, boxDirect.angle, 0, 360, Scalar(255,0,255), 1, LINE_AA);
        
        Point2f vtx[4];
        box.points(vtx);
        for( int j = 0; j < 4; j++ )
            line(cimage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, LINE_AA);
    }

    imshow("result", cimage);
}

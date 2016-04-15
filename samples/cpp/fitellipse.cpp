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
    const int channels = mat.channels();
    
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
                fprintf (stdout, fmt, (*it));
                if(i < (rows)*(cols)-1){
                    fprintf(stdout, ", ");
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
        if (i<srcInfo::max){fprintf (stdout, ",");};
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
        cv::Vec<double, 3> uG{0.235702, 27.0915, 61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};
        
        RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
        rot.LParam.print();
        //  LDist test
        printDist<sWrkInfo::dataType,dst_t>(rot.LDist, "rotLDist");
        printDist<sWrkInfo::dataType,dst_t>(rot.CaDist,"rotCaDist");
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
        cv::Vec<double, 3> uG{0.235702, 27.0915, 61.4471};
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
    
    // Check floating point method.
    if (yesno("Run Floating Point Method Test?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{0.235702, 27.0915, 61.4471};
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
    
    
    // Run Timings
    if (yesno("Run Timings?")) {
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{0.235702, 27.0915, 61.4471};
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

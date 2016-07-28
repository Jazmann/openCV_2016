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
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
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

bool angleComp(double _before, double _theta, double _after){
    double theta, before, after;
    theta = fmod(_theta + CV_PI, 2. * CV_PI);
    if(theta < 0){ theta += 2.0 * CV_PI;}
    before = fmod(_before + CV_PI, 2. * CV_PI);
    if(before < 0){ before += 2.0 * CV_PI;}
    after = fmod(_after + CV_PI, 2. * CV_PI);
    if(after < 0){ after += 2.0 * CV_PI;}
    theta -=  CV_PI;
    before -= CV_PI;
    after -= CV_PI;
    if(before<after){
        return before < theta && theta < after;
    } else {
        return after < theta || theta < before;
    }
}

class Ellipse{
private:
    double angleRad, angleDeg;
public:
    bool radians = true;
    double a, b;
    cv::Point2f center;
    double* angle;
    
    cv::Point2f e0,e1,e2,e3;
    cv::Rect box;
    
    Ellipse( double a, double b, cv::Point2f center, double angle, bool radians = true);
    Ellipse( cv::RotatedRect rect);
    
    void setRadians(bool radQ);
    void setAngle(double ang);
    void setCenter(Point2f cntr);
    void constrainAngle();
    void extrema();
    Point2f pointOnEllipse(double _angle);
    Mat pointsOnArc(double ang1, double ang2, int n);
    Point2f intersectionWithLine(Point2f l0,Point2f l1);
    void print();
    
    
    Point2f pointOnEllipseTopFromX(double x);
    Point2f pointOnEllipseBotFromX(double x);
    Point2f pointOnEllipseLeftFromY(double y);
    Point2f pointOnEllipseRightFromY(double y);
};

Ellipse::Ellipse( double _a, double _b, cv::Point2f _center, double _angle, bool _radians){
    this->center = _center;
    this->a = _a;
    this->b = _b;
    this->setRadians(_radians);
    this->setAngle(_angle);
}

Ellipse::Ellipse( typename cv::RotatedRect rect){
    this->center = rect.center;
    this->a = rect.size.width/2.0;
    this->b = rect.size.height/2.0;
    this->setRadians(false);
    this->setAngle(rect.angle);
}

void Ellipse::setRadians(bool radQ){
    this->radians = radQ;
    if(radQ){
        this->angle = &this->angleRad;
    } else {
        this->angle = &this->angleDeg;
    }
}

void Ellipse::setAngle(double ang){
    if(this->radians){
        this->angleRad = ang;
        this->angleDeg = ang * 180./CV_PI;
        this->angle = &this->angleRad;
    } else {
        this->angleDeg = ang;
        this->angleRad = ang * CV_PI /180.;
        this->angle = &this->angleDeg;
    }
    this->constrainAngle();
    this->extrema();
}

void Ellipse::setCenter(Point2f cntr){
    Point2f trans = cntr - center;
    e0 += trans;
    e1 += trans;
    e2 += trans;
    e3 += trans;
    center = cntr;
}

void Ellipse::constrainAngle(){
    double _a;
    double radMod = fmod(this->angleRad + CV_PI, 2. * CV_PI);
    if(radMod < 0){ radMod += 2.0 * CV_PI;}
    this->angleRad = radMod - CV_PI;
    
    if (this->angleRad > 3. * CV_PI / 4. ) {
        this->angleRad += CV_PI;
    } else if(this->angleRad <  -3. * CV_PI / 4.) {
        this->angleRad -= CV_PI;
    } else if(this->angleRad >  CV_PI / 4. ){
        this->angleRad -= CV_PI/2.;
        _a = this->b;
        this->b = this->a;
        this->a = _a;
    } else if(this->angleRad < -1. *  CV_PI / 4. ){
        this->angleRad += CV_PI/2.;
        _a = this->b;
        this->b = this->a;
        this->a = _a;
    }
    this->angleDeg = this->angleRad * 180./CV_PI;
}

Point2f Ellipse::pointOnEllipse(double _ang){
    Point2f out;
    double ang;
    if(this->radians){
        ang = _ang;
    } else {
        ang = _ang * CV_PI /180.;
    }
    double sqrtD = std::sqrt((b*b)*(std::cos(ang) * std::cos(ang) ) + (a*a)*(std::sin(ang) * std::sin(ang) ));
    
    out.x = this->center.x + (a*b*std::cos(ang)*std::cos(this->angleRad))/sqrtD - (a*b*std::sin(ang)*std::sin(this->angleRad))/sqrtD;
    
    out.y = this->center.y + (a*b*std::sin(ang)*std::cos(this->angleRad))/sqrtD + (a*b*std::cos(ang)*std::sin(this->angleRad))/sqrtD;
    
    return out;
}


Mat Ellipse::pointsOnArc(double _ang1, double _ang2, int n){
    Mat out(n,2,CV_32F);
    double ang1, ang2, angStep;
    if(this->radians){
        ang1 = _ang1; ang2 = _ang2;
    } else {
        ang1 = _ang1 * CV_PI /180.;
        ang2 = _ang2 * CV_PI /180.;
    }
    angStep = (ang2-ang1)/(n-1);
    for (int i=0; i<n; i += 1) {
        double ang = ang1 + i * angStep;
        Point2f pnt = this->pointOnEllipse(ang);
        out.at<float>(i,0) = pnt.x;
        out.at<float>(i,1) = pnt.y;
    }
    
    return out;
}


void  Ellipse::extrema(){
    double theta2 = fmod(angleRad, CV_PI/2.);
    double cosT = std::cos(theta2), sinT = std::sin(theta2), tanT = std::tan(theta2);
    double xDiff1 = ((a - b)*(a + b)*sinT)/sqrt((b*b) + (a*a)*(tanT*tanT)); // +ve if a > b
    double yDiff2 = ((a - b)*(a + b)*sinT)/sqrt((a*a) + (b*b)*(tanT*tanT)); // +ve if a > b
    double xDiff2 = cosT*sqrt((a*a) + (b*b)*(tanT*tanT));
    double yDiff1 = cosT*sqrt((b*b) + (a*a)*(tanT*tanT));
    e0 = Point2f(center.x + xDiff1,center.y + yDiff1);
    e1 = Point2f(center.x - xDiff1,center.y - yDiff1);
    e2 = Point2f(center.x - xDiff2,center.y - yDiff2);
    e3 = Point2f(center.x + xDiff2,center.y + yDiff2);
    if(b > a){
        xDiff1 *= -1.;
        yDiff2 *= -1.;
    }
    double xMin, xMax, yMin, yMax;
    if(xDiff1 > xDiff2){
        xMax = center.x + xDiff1;
        xMin = center.x - xDiff1;
    } else {
        xMax = center.x + xDiff2;
        xMin = center.x - xDiff2;
    }
    if(yDiff1 > yDiff2){
        yMax = center.y + yDiff1;
        yMin = center.y - yDiff1;
    } else {
        yMax = center.y + yDiff2;
        yMin = center.y - yDiff2;
    }
    box = Rect(xMin,yMin,xMax-xMin,yMax-yMin);
}


Point2f Ellipse::pointOnEllipseTopFromX(double x){
    double cosT = std::cos(angleRad), sinT = std::sin(angleRad);
    double cos2T = std::cos(2.* angleRad), sin2T = std::sin(2.* angleRad);
    double cosT2 = cosT*cosT, sinT2 = sinT*sinT;
    double denomX = (a*a) + (b*b) + (a-b)*(a+b)*cos2T;
    double sqrX   = denomX- 2*(center.x - x)*(center.x - x);
    
    double y = (2*(a*a)*center.y*cosT2 + a * b * sqrt(2.*sqrX) + 2*(b*b)*center.y*sinT2 + (b-a)*(a+b)*(center.x - x)*sin2T)/denomX;
    
    return Point2f(x,y);
    }


Point2f Ellipse::pointOnEllipseBotFromX(double x){
    double cosT = std::cos(angleRad), sinT = std::sin(angleRad);
    double cos2T = std::cos(2.* angleRad), sin2T = std::sin(2.* angleRad);
    double cosT2 = cosT*cosT, sinT2 = sinT*sinT;
    double denomX = (a*a) + (b*b) + (a-b)*(a+b)*cos2T;
    double sqrX   = denomX- 2*(center.x - x)*(center.x - x);
    
    double y = (2*(a*a)*center.y*cosT2 - a * b * sqrt(2.*sqrX) + 2*(b*b)*center.y*sinT2 + (b-a)*(a+b)*(center.x - x)*sin2T)/denomX;
    
    return Point2f(x,y);
}


Point2f Ellipse::pointOnEllipseLeftFromY(double y){
    double x, cosT, cos2T, cosT2, sinT, sin2T, sinT2, denomY, sqrY;
    cosT = std::cos(angleRad), sinT = std::sin(-1*angleRad);
    cos2T = std::cos(2.* angleRad), sin2T = std::sin(-2.* angleRad);
    cosT2 = cosT*cosT, sinT2 = sinT*sinT;
    denomY = (a*a) + (b*b) + ((b*b)-(a*a))*cos2T;
    sqrY   = denomY - 2*(center.y - y)*(center.y - y) ;
    
    if(sqrY>=0.0){
        x = (2*center.x*((b*b)*cosT2+(a*a)*sinT2)   + (a - b)*(a + b)*(center.y - y)*sin2T - a*b*sqrt(2.*sqrY) )/denomY;
    } else {
        x = box.tl().x;
    }
    
  //  double x = 2*center.x*( (b*b)*cosT2 + (a*a)*sinT2 + (a-b)*(a+b)*(center.y - y)*sin2T)/denomY - a * b * sqrt(2.*sqrY) ;
    
    return Point2f(x,y);
}

Point2f Ellipse::pointOnEllipseRightFromY(double y){
    double x, cosT, cos2T, cosT2, sinT, sin2T, sinT2, denomY, sqrY;
    cosT  = std::cos(angleRad);     sinT  = std::sin(-1 * angleRad);
    cos2T = std::cos(2.* angleRad); sin2T = std::sin(-2.* angleRad);
    cosT2 = cosT*cosT;              sinT2 = sinT*sinT;
    denomY = (a*a) + (b*b) + ((b*b)-(a*a))*cos2T;
    sqrY   = denomY - 2*(center.y - y)*(center.y - y) ;
    if(sqrY>=0.0){
        x = (2*center.x*((b*b)*cosT2+(a*a)*sinT2)   + (a - b)*(a + b)*(center.y - y)*sin2T + a*b*sqrt(2.*sqrY) )/denomY;
    } else {
        x = box.br().x;
    }
    
    return Point2f(x,y);
}

Point2f Ellipse::intersectionWithLine(Point2f l0,Point2f l1){
    double cosT = std::cos(angleRad), sinT = std::sin(angleRad);
    Point2f ll0 = Point2f((l0.x - center.x) * cosT + (l0.y - center.y) * sinT,
                          (l0.y - center.y) * cosT - (l0.x - center.x) * sinT);
    Point2f ll1 = Point2f((l1.x - center.x) * cosT + (l1.y - center.y) * sinT,
                          (l1.y - center.y) * cosT - (l1.x - center.x) * sinT);
    Point2f vec = ll1-ll0;
    double denom = ((b*b)*(vec.x * vec.x) + (a*a)*(vec.y * vec.y));
    double rt = (b*b)*(vec.x * vec.x) + (a*a)*(vec.y * vec.y) - (vec.y*ll0.x - vec.x*ll0.y)*(vec.y*ll0.x - vec.x*ll0.y);
    if(rt>0.){
        double absqrt = a*b*sqrt(rt);
        double d1 = -1.*((b*b) * vec.x * ll0.x + (a*a) * vec.y * ll0.y + absqrt)/denom;
        double d2 = (-1.*(b*b) * vec.x * ll0.x - (a*a) * vec.y * ll0.y + absqrt)/denom;
        Point2f out;
    if(d1>0 && d1 > d2){
        out = l0 + d1 * (l1-l0);
    } else {
        out = l0 + d2 * (l1-l0);
    }
        return out;
    } else {
        return Point2f(0,0);
    }
}

void Ellipse::print(){
    fprintf (stdout, "Graphics[{\n{ Point[{%f, %f}],Point[{ %f, %f}],Point[{ %f, %f}],Point[{ %f, %f}] },\n",this->e0.x, this->e0.y,this->e1.x, this->e1.y,this->e2.x, this->e2.y,this->e3.x, this->e3.y);
    fprintf (stdout, "FaceForm[None],EdgeForm[Black],ellipse[{ {%f, %f},{ %f, %f}, %f }]\n}]\n",this->center.x, this->center.y, this->a, this->b, *this->angle);
}

template<int src_t> static void findFrameOrientation(Mat img, Point * midPnt, Point * perpVec){
    
    int length = 0, len;
    Point end, start, vec, longStart, longEnd;
    img.type();
    
    end = Point(0,0); start = Point(0,0); vec = Point(1,0);
    
    length = runReachLongest<src_t>(img, &start, &end, vec);
    longStart = start; longEnd = end;
    *perpVec = Point(0,1);
    
    end = Point(0,0); start = Point(0,img.cols-1); vec = Point(1,0);
    
    len = runReachLongest<src_t>(img, &start, &end, vec);
    if (len > length) {
        length = len;
        longStart = start; longEnd = end;
        *perpVec = Point(0,-1);
    }
    
    end = Point(0,0); start = Point(0,0); vec = Point(0,1);
    
    len = runReachLongest<src_t>(img, &start, &end, vec);
    if (len > length) {
        length = len;
        longStart = start; longEnd = end;
        *perpVec = Point(1,0);
    }
    
    end = Point(0,0); start = Point(img.rows-1,0); vec = Point(0,1);
    
    len = runReachLongest<src_t>(img, &start, &end, vec);
    if (len > length) {
        length = len;
        longStart = start; longEnd = end;
        *perpVec = Point(-1,0);
    }
    
    midPnt->x = (longStart.x+longEnd.x)/2.0;
    midPnt->y = (longStart.y+longEnd.y)/2.0;

}

template<typename T> void meanMedian(std::vector<T> vals, double frac, double * mm, double * tol){
    double u=0.0, l=0.0, total=0.0;
    std::sort(vals.begin(), vals.end());
    
    int indxL=int(((1.0-frac)/2.0)*vals.size());
    int indxH=int(((1.0+frac)/2.0)*vals.size());
    
    for (int i=indxL; i < indxH; ++i) {
        total += vals[i];
    }
    
    * mm = total/(indxH-indxL);
    
    u = *mm-vals[indxL];
    l = vals[indxH]-*mm;
    * tol = u > l? u : l;
}

enum pointClass:CV_8U_TYPE{unclassified, tip, distal, middle, onFrame, anomaly};

class graveShape{
private:
public:
    Point2f d00, d01, d10, d11, line0, line1;
    Point2f eL, eR, eT, eB; // the extreme edge points: eL = Left, eR = Right, eT = Top, eB = Bottom.
    Ellipse curve;
    double theta0, theta1; // The angles from the center of the ellipse to the trapezium edges.
    Point orient; // The frame orientation. Vector from base to tip {0,1} {0,-1} {1,0} {-1,0}
    Rect box;
    
    
    graveShape(Point2f d00, Point2f d01, Point2f d10, Point2f d11, Ellipse curve);
    
    void findThetas();
    void findAxis();
    void findOrient();
    void findExtremePoints();
    void findBoundingBox();
    
    void translate(Point2f shift);
    void rotate(double theta);
    
    void alignCurveToTrapezium();
    
    Mat pointsOnLeft();
    Mat pointsOnRight();
    
    void print(const char* name);
};


void graveShape::print(const char* name){
    fprintf (stdout, "%sTrap = { {{%f, %f},{ %f, %f}},{{ %f, %f},{ %f, %f} }};\n",name,d00.x, d00.y, d01.x, d01.y, d10.x, d10.y, d11.x, d11.y);
    fprintf (stdout, "%sEllipse = { {%f, %f},{ %f, %f}, %f };\n",name,this->curve.center.x, this->curve.center.y, this->curve.a, this->curve.b, *this->curve.angle);
    fprintf (stdout, "eL = {%f, %f}; eR = { %f, %f}; eT = { %f, %f}; eB = { %f, %f};\n",this->eL.x, this->eL.y, this->eR.x, this->eR.y,
             this->eT.x, this->eT.y,this->eB.x, this->eB.y);
    fprintf (stdout, "e0 = {%f, %f}; e1 = { %f, %f}; e2 = { %f, %f}; e3 = { %f, %f};\n",this->curve.e0.x, this->curve.e0.y, this->curve.e1.x, this->curve.e1.y,
             this->curve.e2.x, this->curve.e2.y,this->curve.e3.x, this->curve.e3.y);
    fprintf (stdout, "%sBox = Rectangle[{%d, %d},{ %d, %d}];\n",name,box.tl().x, box.tl().y, box.br().x, box.br().y);
    fprintf (stdout, "%sTheta = {%f, %f};\n",name,theta0, theta1);
    fprintf (stdout, "%sOrient = { %d, %d};\n",name,orient.x,orient.y);
    fprintf (stdout, "%sAxis = Line[{{%f, %f},{ %f, %f}}];\n",name,line0.x,line0.y, line1.x, line1.y);
    Mat mdlPntsL = pointsOnLeft();
    char buf[100];
    strcpy(buf, name);
    strcat(buf, "MdlPntsL");
    printImg<CV_32F>(mdlPntsL, buf);
    Mat mdlPntsR = pointsOnRight();
    char buff[100];
    strcpy(buff, name);
    strcat(buff, "MdlPntsR");
    printImg<CV_32F>(mdlPntsR, buff);
    fprintf (stdout, "Graphics[{FaceForm[None], EdgeForm[Black], \
                       ellipse[%sEllipse],modelToGraphics[{%sTrap, %sEllipse}, {{0, 0}, 0}], \
                       {FaceForm[None], EdgeForm[Black], %sBox}, \
                       {%sAxis, Green, Point[eL], Point[eR], Point[eT], Point[eB]}, Map[Point, %sMdlPntsL], Map[Point, %sMdlPntsR]}, Frame -> True]\n",name,name,name,name,name,name,name);
}


graveShape::graveShape(Point2f _d00, Point2f _d01, Point2f _d10, Point2f _d11, Ellipse _curve):d00(_d00),d01(_d01),d10(_d10),d11(_d11),curve(_curve){
    findThetas();
    findAxis();
    findOrient();
    findExtremePoints();
    findBoundingBox();
};

void graveShape::findThetas(){
    Point2f vec;
    vec = d01 - curve.center;
    theta0 = atan2(vec.y, vec.x);
    vec = d11 - curve.center;
    theta1 = atan2(vec.y, vec.x);
};


void graveShape::findAxis(){
    line0 = (d00+d10)/2;
    line1 = (d01+d11)/2;
    line1 = curve.intersectionWithLine(line0, line1);
};

void graveShape::findOrient(){
    Point2f vec = line1-line0;
    if(abs(vec.x)>abs(vec.y)){
        orient = Point(int(vec.x/abs(vec.x)),0);
    } else {
        orient = Point(0,int(vec.y/abs(vec.y)));
    };
};

void graveShape::findExtremePoints(){
    Point2f e[8];
    int i=3;
    e[0]=d00; e[1]=d01;e[2]=d10;e[3]=d11;
    float e0Theta = atan2(curve.e0.y-curve.center.y,curve.e0.x-curve.center.x);
    float e1Theta = atan2(curve.e1.y-curve.center.y,curve.e1.x-curve.center.x);
    float e2Theta = atan2(curve.e2.y-curve.center.y,curve.e2.x-curve.center.x);
    float e3Theta = atan2(curve.e3.y-curve.center.y,curve.e3.x-curve.center.x);
    if(angleComp(theta1,e0Theta,theta0)){
        i++;
        e[i] = curve.e0;
    }
    if(angleComp(theta1,e1Theta,theta0)){
        i++;
        e[i] = curve.e1;
    }
    if(angleComp(theta1,e2Theta,theta0)){
        i++;
        e[i] = curve.e2;
    }
    if(angleComp(theta1,e3Theta,theta0)){
        i++;
        e[i] = curve.e3;
    }
    // Add debugging warning if j>6. There should be at most 2 extrema on the curve.
    double xMin=e[0].x,xMax=e[0].x,yMin=e[0].y,yMax=e[0].y;
    eL = e[0]; eR = e[0]; eB = e[0]; eT = e[0];
    for (int j=1; j<=i; j++) {
        if(e[j].x<xMin){xMin = e[j].x; eL = e[j];} else if(e[j].x>xMax){ xMax = e[j].x; eR = e[j];}
        if(e[j].y<yMin){yMin = e[j].y; eB = e[j];} else if(e[j].y>yMax){ yMax = e[j].y; eT = e[j];}
    }
    
};

void graveShape::findBoundingBox(){
    int xMin = floor(eL.x), xMax = ceil(eR.x);
    int yMin = floor(eB.y), yMax = ceil(eT.y);
    box = Rect(xMin,yMin,xMax-xMin,yMax-yMin);
};

void graveShape::alignCurveToTrapezium(){
    // fix ellipse to trapezium pnts.
    Mat fivePnts(5,2,CV_32F);
    fivePnts = curve.pointsOnArc(theta0,theta1,5);
    fivePnts.at<float>(0,0) = d01.x;
    fivePnts.at<float>(0,1) = d01.y;
    fivePnts.at<float>(4,0) = d11.x;
    fivePnts.at<float>(4,1) = d11.y;
    
    RotatedRect rect = fitEllipseDirect(fivePnts);
    curve = Ellipse(rect);
    curve.setRadians(true);
    
};

void graveShape::translate(Point2f shift){
    curve.setCenter(curve.center + shift);
    d00 += shift; d01 += shift;
    d10 += shift; d11 += shift;
    eL += shift; eR += shift;
    eT += shift; eB += shift;
    findBoundingBox();
}

void graveShape::rotate(double theta){
    Matx<double,2,2> Rot = {std::cos(theta), -1.*std::sin(theta), std::sin(theta), std::cos(theta) };
    Point2f center = curve.center, temp;
    translate(-1. * center);
    curve.setAngle(*curve.angle + theta);
    temp.x = Rot(0,0) * d00.x + Rot(0,1) * d00.y;
    temp.y = Rot(1,0) * d00.x + Rot(1,1) * d00.y;
    d00 = temp;
    temp.x = Rot(0,0) * d01.x + Rot(0,1) * d01.y;
    temp.y = Rot(1,0) * d01.x + Rot(1,1) * d01.y;
    d01 = temp;
    temp.x = Rot(0,0) * d10.x + Rot(0,1) * d10.y;
    temp.y = Rot(1,0) * d10.x + Rot(1,1) * d10.y;
    d10 = temp;
    temp.x = Rot(0,0) * d11.x + Rot(0,1) * d11.y;
    temp.y = Rot(1,0) * d11.x + Rot(1,1) * d11.y;
    d11 = temp;
  //  temp.x = Rot(0,0) * center.x + Rot(0,1) * center.y;
  //  temp.y = Rot(1,0) * center.x + Rot(1,1) * center.y;
  //  center = temp;
    translate(center);
    findThetas();
    findAxis();
    findOrient();
    findExtremePoints();
    findBoundingBox();
}

Mat graveShape::pointsOnLeft(){
    Point2f pnt = d00;
    if(orient.x==0){ // vertical
        float start = d00.y;
        float mid   = d01.y;
        if(orient.y>0){ // up
            float end = eT.y;
            int n = ceil(end-start)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d01.x - d00.x)/(d01.y - d00.y);
            float x = d00.x;
            for(float y=start;y<mid; y++){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                x += m;
                indx++;
            }
            for(double y=mid;y<end; y++){
                pnt=curve.pointOnEllipseLeftFromY(y);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        } else { // down
            float end = eB.y;
            int n = ceil(start-end)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d01.x - d00.x)/(d01.y - d00.y);
            float x = d00.x;
            for(float y=start;y>mid; y--){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                x -= m;
                indx++;
            }
            for(double y=mid;y>end; y--){
                pnt=curve.pointOnEllipseRightFromY(y);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        }
        
    } else { // horizontal
        float start = d00.x;
        float mid   = d01.x;
        if(orient.x>0){ // right
            float end = eR.x;
            int n = ceil(end-start)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d01.y-d00.y)/(d01.x - d00.x);
            float y = d00.y;
            for(float x=start;x<mid; x++){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                y += m;
                indx++;
            }
            for(double x=mid;x<end; x++){
                pnt=curve.pointOnEllipseTopFromX(x);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        } else { // left
            float end = eL.x;
            int n = ceil(start-end)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d01.y-d00.y)/(d01.x - d00.x);
            float y = d00.y;
            for(float x=start;x>mid; x--){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                y -= m;
                indx++;
            }
            for(double x=mid;x>end; x--){
                pnt=curve.pointOnEllipseBotFromX(x);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        }

    }
}

Mat graveShape::pointsOnRight(){
    Point2f pnt = d10;
    if(orient.x==0){ // vertical
        float start = d10.y;
        float mid   = d11.y;
        if(orient.y>0){ // up
            float end = eT.y;
            int n = ceil(end-start)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d11.x - d10.x)/(d11.y - d10.y);
            float x = d10.x;
            for(float y=start;y<mid; y++){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                x += m;
                indx++;
            }
            for(double y=mid;y<end; y++){
                pnt=curve.pointOnEllipseRightFromY(y);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        } else { // down
            float end = eB.y;
            int n = ceil(start-end)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d11.x - d10.x)/(d11.y - d10.y);
            float x = d10.x;
            for(float y=start;y>mid; y--){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                x -= m;
                indx++;
            }
            for(double y=mid;y>end; y--){
                pnt=curve.pointOnEllipseLeftFromY(y);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        }
        
    } else { // horizontal
        float start = d10.x;
        float mid   = d11.x;
        if(orient.x>0){ // right
            float end = eR.x;
            int n = ceil(end-start)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d11.y-d10.y)/(d11.x - d10.x);
            float y = d10.y;
            for(float x=start;x<mid; x++){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                y += m;
                indx++;
            }
            for(double x=mid;x<end; x++){
                pnt=curve.pointOnEllipseBotFromX(x);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        } else { // left
            float end = eL.x;
            int n = ceil(start-end)+1;
            Mat out(n,2,CV_32F);
            int indx=0;
            
            float m = (d11.y-d10.y)/(d11.x - d10.x);
            float y = d10.y;
            for(float x=start;x>mid; x--){
                out.at<float>(indx,0) = x;
                out.at<float>(indx,1) = y;
                y -= m;
                indx++;
            }
            for(double x=mid;x>end; x--){
                pnt=curve.pointOnEllipseTopFromX(x);
                out.at<float>(indx,0) = pnt.x;
                out.at<float>(indx,1) = pnt.y;
                indx++;
            }
            if(indx==n){
                return out;
            } else {
                out.resize(indx);
                return out;
            }
            
        }
        
    }
}


class fingerTipModel{
private:
public:
    graveShape mdl, imgMdl;
    double width; // The width of the Distal finger part.
    
    // Position and orientation of the model in the image.
    // Updated by supplying edge and midline points found by fillament fill.
    Point2f pos;
    double angle;
    Point orient; // The frame orientation. Vector from base to tip {0,1} {0,-1} {1,0} {-1,0}
    
    fingerTipModel(Point2f d00, Point2f d01, Point2f d10, Point2f d11, Ellipse curve);
    
    void findWidths();
    
    void updateImageModel();
    
    void translateModel(Point2f shift);
    void rotateModel(double theta);
    
    void alignToEllipseCenter();
    void alignToTrapezium();
    void allignToAreaCentroid();
    
    void print();
};


void fingerTipModel::print(){
    fprintf (stdout, "pos= {%f, %f};\n",pos.x, pos.y);
    fprintf (stdout, "angle = %f;\n",angle);
    mdl.print("mdl");
    imgMdl.print("imgMdl");
}


fingerTipModel::fingerTipModel(Point2f _d00, Point2f _d01, Point2f _d10, Point2f _d11, Ellipse _curve):mdl(_d00,_d01,_d10,_d11,_curve),imgMdl(_d00,_d01,_d10,_d11,_curve){
    
    pos = Point2f(0,0);
    angle = 0.0;
    
    findWidths();
};


void fingerTipModel::findWidths(){
    Point2f vec = ((mdl.d00-mdl.d10)+(mdl.d01-mdl.d11))/2.;
    width = std::sqrt((vec.x * vec.x)+(vec.y * vec.y));
};


void fingerTipModel::updateImageModel(){
    imgMdl = mdl;
    imgMdl.rotate(angle);
    imgMdl.translate(pos);
}

void fingerTipModel::translateModel(Point2f shift){
    mdl.translate(shift);
    imgMdl.translate(-1*shift);
    pos -= shift;
}

void fingerTipModel::rotateModel(double theta){
    mdl.rotate(theta);
    imgMdl.rotate(-1*theta);
    angle -= theta;
}

void fingerTipModel::alignToEllipseCenter(){
    translateModel(-1. * mdl.curve.center);
    rotateModel(-1. * (*mdl.curve.angle));
};

void fingerTipModel::alignToTrapezium(){
    Point2f vec = mdl.line1-mdl.line0;
    double theta = atan2(vec.y,vec.x);
    translateModel(-1. * mdl.line1);
    rotateModel(-1. * theta);
};



int classifyPointOnFrame(InputArray _img, InputArray _edgePnts, InputArray _midPnts, InputOutputArray _edgePntsClass, InputOutputArray _midPntsClass){
    
    Mat img = _img.getMat();
    Mat edgePnts = _edgePnts.getMat();
    Mat midPnts = _midPnts.getMat();
    Mat edgePntsClass = _edgePntsClass.getMat();
    Mat midPntsClass = _midPntsClass.getMat();
    
    int count = midPnts.rows;
    int rows = img.rows, cols = img.cols;
    
    int frameCount=0;
    for (int i=0; i<count; i++) {
        int j = 2*count-i-1;
        Point top = edgePnts.at<Point>(i,0);
        Point bot = edgePnts.at<Point>(j,0);
        Point mid = midPnts.at<Point>(i,0);
        
        bool topOnFrameQ = top.x==0||top.x==rows-1||top.y==0||top.y==cols-1;
        bool botOnFrameQ = bot.x==0||bot.x==rows-1||bot.y==0||bot.y==cols-1;
        
        if (topOnFrameQ) {
            edgePntsClass.at<pointClass>(i,0) = onFrame;
            midPntsClass.at<pointClass>(i,0)  = onFrame;
            frameCount++;
            
        }
        if (botOnFrameQ) {
            edgePntsClass.at<pointClass>(j,0) = onFrame;
            midPntsClass.at<pointClass>(i,0)  = onFrame;
            frameCount++;
            
        }
    }
    
    printImg<CV_8U>(edgePntsClass,"edgePntsClass");
    printImg<CV_8U>(midPntsClass,"midPntsClass");
    
    return frameCount;
}


void classifyPointInitial(Point perpVec, InputArray _edgePnts, InputArray _midPnts, InputOutputArray _edgePntsClass, InputOutputArray _midPntsClass, int tipCount, int distalCount){
    
    Mat edgePnts = _edgePnts.getMat();
    Mat midPnts = _midPnts.getMat();
    Mat edgePntsClass = _edgePntsClass.getMat();
    Mat midPntsClass = _midPntsClass.getMat();
    
    int count = midPnts.rows;
    
    std::vector<int> filLengths;
    filLengths.reserve(count);
    
    for (int i=0; i<count; i++) {
        int j = 2*count-i-1;
        Point top = edgePnts.at<Point>(i,0);
        Point bot = edgePnts.at<Point>(j,0);
        Point mid = midPnts.at<Point>(i,0);
        
        if (midPntsClass.at<pointClass>(i,0)  != onFrame ) {
            if (perpVec.x==0) {
                filLengths.push_back(std::abs(top.x-bot.x));
            } else {
                filLengths.push_back(std::abs(top.y-bot.y));
            }
        }
        
    }
    
    Mat fillLengthsM(filLengths);
    
    double mm, tol;
    meanMedian<int>(filLengths, 0.33, &mm, &tol);
    
    // tip Classification.
    tipCount = 0;
    distalCount = 0;
    int offTipRun = 0;
    
    for (int i=count-1; i >= 0; --i) {
        int j = 2*count-i-1;
        Point top = edgePnts.at<Point>(i,0);
        Point bot = edgePnts.at<Point>(j,0);
        Point mid = midPnts.at<Point>(i,0);
        
        if (edgePntsClass.at<pointClass>(i,0) != unclassified||edgePntsClass.at<pointClass>(j,0) != unclassified||midPntsClass.at<pointClass>(i,0) != unclassified) {
            continue;
        }
        
        bool onTipQ=false, onDistalQ=false;
        double dist;
        if (perpVec.x==0) {
            dist = (std::abs(top.x-bot.x));
        } else {
            dist = (std::abs(top.y-bot.y));
        }
        if(dist  < mm-tol && offTipRun < 7){
            onTipQ=true;
            offTipRun = 0;
        } else if(dist > mm-tol && dist < mm+tol){
            onDistalQ=true;
            offTipRun++;
        } else {
            
            offTipRun++;
        }
        
        if (onTipQ) {
            edgePntsClass.at<pointClass>(i,0) = tip;
            edgePntsClass.at<pointClass>(j,0) = tip;
            midPntsClass.at<pointClass>(i,0)  = tip;
            tipCount++;
            
        } else if(onDistalQ){
            edgePntsClass.at<pointClass>(i,0) = distal;
            edgePntsClass.at<pointClass>(j,0) = distal;
            midPntsClass.at<pointClass>(i,0)  = distal;
            distalCount++;
        } else {
            edgePntsClass.at<pointClass>(i,0) = anomaly;
            edgePntsClass.at<pointClass>(j,0) = anomaly;
            midPntsClass.at<pointClass>(i,0)  = anomaly;
            
        }
    }
    
}

template<typename T> Mat transformPoints(InputArray _pnts, Point origin, double angle){
    
    Mat pnts = _pnts.getMat();
    int count = pnts.rows;
    
    // find rotation
    Matx<double,2,2> Rot = {std::cos(angle), -1.*std::sin(angle), std::sin(angle), std::cos(angle) };
    
    // Put all pnts into neutral orientation origin at line(2) rotated by -digitTheta.
    Mat pntsN(pnts.rows, pnts.cols, pnts.type());
    
    for (int i=0; i<count; i++) {
        Point_<T> pnt = pnts.at<Point_<T>>(i,0);
        
        pntsN.at<T>(i,0) = Rot(0,0) * (pnt.x - origin.x) + Rot(0,1) * (pnt.y - origin.y);
        pntsN.at<T>(i,1) = Rot(1,0) * (pnt.x - origin.x) + Rot(1,1) * (pnt.y - origin.y);
    }
    return pntsN;
}

template<typename T> Mat iTransformPoints(InputArray _pntsN, Point origin, double angle){
    
    Mat pntsN = _pntsN.getMat();
    int count = pntsN.rows;
    
    // find rotation
    Matx<double,2,2> Rot = {std::cos(-1.*angle), -1.*std::sin(-1.*angle), std::sin(-1.*angle), std::cos(-1.*angle) };
    
    // Put all pnts into neutral orientation origin at line(2) rotated by -digitTheta.
    Mat pnts(pntsN.rows, pntsN.cols, pntsN.type());
    
    for (int i=0; i<count; i++) {
        Point_<T> pnt = pntsN.at<Point_<T>>(i,0);
        
        pnts.at<T>(i,0) = Rot(0,0) * pnt.x + Rot(0,1) * pnt.y + origin.x;
        pnts.at<T>(i,1) = Rot(1,0) * pnt.x + Rot(1,1) * pnt.y + origin.y;
    }
    return pnts;
}

Mat extractPntsClassifiedAs(pointClass cls, InputArray _pntsIn, InputArray _pntsClass){
    
    Mat pntsIn = _pntsIn.getMat();
    Mat pntsClass = _pntsClass.getMat();
    
    int count = pntsIn.rows;
    
    Mat pointsOut;
    pointsOut.reserve(count);
    
    for (int i=0; i<count; i++) {
        
        if (pntsClass.at<pointClass>(i,0)  == cls ) {
            pointsOut.push_back(pntsIn.row(i));
        }
        
    }
    
    return pointsOut;
}


Mat extractPntsNotClassifiedAs(pointClass cls, InputArray _pntsIn, InputArray _pntsClass){
    
    Mat pntsIn = _pntsIn.getMat();
    Mat pntsClass = _pntsClass.getMat();
    
    int count = pntsIn.rows;
    
    Mat pointsOut;
    pointsOut.reserve(count);
    
    for (int i=0; i<count; i++) {
        
        if (pntsClass.at<pointClass>(i,0)  != cls ) {
            pointsOut.push_back(pntsIn.row(i));
        }
        
    }
    return pointsOut;
}



Mat trapeziumFit(InputArray _line, InputArray _pnts){
    
    Mat pnts = _pnts.getMat();
    Matx<double,2,2> line = _line.getMat();
    
    int count = pnts.rows;
    
    // find orientation
    double digitTheta = std::atan2((line(1,1)-line(0,1)),(line(1,0)-line(0,0)));
    
    Mat pntsN = transformPoints<int>(pnts, Point(line(1,0),line(1,1)), -1.*digitTheta);
    
    Mat pntsAbsN(count, 2, CV_32SC1);
    
    int xMin = pntsN.at<int>(0,0), xMax = pntsN.at<int>(count-1,0);
    
    for (int i=0; i<pntsN.rows; i++) {
        int x1 = pntsN.at<int>(i,0);
        if(xMin>x1){xMin=x1;}; if(xMax<x1){xMax=x1;};
        pntsAbsN.at<int>(i,0) = x1;
        pntsAbsN.at<int>(i,1) = abs(pntsN.at<int>(i,1));
    }
    
    cv::Mat  lineTop(1,4,CV_32F);
    cv::fitLine(pntsAbsN, lineTop, CV_DIST_L2, 0, 0.01, 0.01);
    
    Mat fitLineN(4,2,CV_32F);
    fitLineN.at<float>(0,0) = float(xMin);
    fitLineN.at<float>(0,1) = (lineTop.at<float>(0) * lineTop.at<float>(3) + lineTop.at<float>(1) *(fitLineN.at<float>(0,0) - lineTop.at<float>(2) ))/lineTop.at<float>(0);
    
    fitLineN.at<float>(1,0) = float(xMax);
    fitLineN.at<float>(1,1) = (lineTop.at<float>(0) * lineTop.at<float>(3) + lineTop.at<float>(1) *(fitLineN.at<float>(1,0) - lineTop.at<float>(2) ))/lineTop.at<float>(0);
    
    fitLineN.at<float>(2,0) =     fitLineN.at<float>(0,0);
    fitLineN.at<float>(2,1) = -1.*fitLineN.at<float>(0,1);
    
    fitLineN.at<float>(3,0) =     fitLineN.at<float>(1,0);
    fitLineN.at<float>(3,1) = -1.*fitLineN.at<float>(1,1);
    Mat fitLineOut = iTransformPoints<float>(fitLineN, Point(line(1,0),line(1,1)), -1.*digitTheta);
    return fitLineOut;
}



static cv::Point2f thisPointOnLine(float *line, cv::Point2f pnt)
{
    cv::Point2f out;
    out.x =  line[2] + (line[0]*(line[0]*(pnt.x - line[2]) + line[1]*(pnt.y - line[3])))/(line[0]*line[0] + line[1]*line[1]);
    out.y = (line[1] * (line[0]*(pnt.x - line[2]) + line[1]*(pnt.y - line[3])))/(line[0]*line[0] + line[1]*line[1]) + line[3];
    return out;
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
        
        Mat pnts, middlePnts;
        fillamentFill<CV_8UC4>(LCaCbImg, pnts, middlePnts, start, end);
        printImg<CV_32SC1>(pnts,"pnts");
        
        end = runReachToEnd<CV_8UC4>(LCaCbImg,  start, Point(-1,1));
        fprintf (stdout, "runReachToEnd ={{%d,%d},{%d, %d }}\n", start.x, start.y, end.x, end.y);
        
        Mat midPnts = runReachMidlineMat<CV_8UC4>(LCaCbImg,  start, Point(-1,0));
        printImg<CV_32SC1>(midPnts,"midPnts");
        
        
    //    cv::Mat3b roiMat = LCaCbImg(cv::Rect(88,163,127,60));// topLeft.x topLeft.y Width height
    //    cv::Scalar mean;
    //    mean =  cv::mean(roiMat);
    //    fprintf (stdout, "mean ={%f, %f, %f }\n", mean[0], mean[1], mean[2]);
        
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
        
        cv::Matx<double,3,2> line;
        
        if (yesno("Run KinkFit Test0?")) {
    cv::Matx<double,103,2> testPnts{       45.,52.888151769357144,45.61290322580645,53.379903331559454,46.225806451612904,
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
    
        kinkFitLine( testPnts,  line, CV_DIST_L2, 0, 0.01, 0.01 );
        fprintf (stdout, "line ={{%f, %f}, {%f, %f},{%f, %f}}\n", line(0,0), line(0,1), line(1,0), line(1,1), line(2,0), line(2,1));
        };
        
        if (yesno("Run KinkFit TestA?")) {
            
            cv::Matx<double,102,2> testPntsA{31.,633.,32.,645.,33.,686.,34.,701.,35.,719.,36.,713.,38.,659.,39.,733.,40.,678.,41.,745.,42.,772.,43.,695.,44.,738.,45.,737.,46.,768.,47.,770.,49.,744.,50.,757.,51.,821.,52.,808.,53.,791.,54.,789.,55.,811.,56.,809.,57.,864.,58.,849.,60.,817.,61.,843.,62.,854.,63.,850.,64.,912.,65.,901.,66.,896.,67.,846.,68.,888.,69.,922.,71.,939.,72.,934.,73.,944.,74.,946.,75.,973.,76.,947.,76.,973.,83.,1003.,90.,879.,97.,947.,104.,945.,111.,945.,118.,938.,125.,972.,132.,968.,139.,910.,146.,907.,153.,930.,160.,954.,167.,933.,174.,895.,181.,947.,188.,888.,195.,879.,202.,881.,209.,878.,216.,928.,223.,923.,230.,834.,237.,902.,244.,835.,251.,923.,258.,854.,265.,853.,272.,897.,279.,908.,286.,824.,293.,863.,300.,860.,307.,859.,314.,897.,321.,874.,328.,830.,335.,867.,342.,820.,349.,816.,356.,899.,363.,827.,370.,823.,377.,834.,384.,845.,391.,857.,398.,801.,405.,829.,412.,837.,419.,877.,426.,804.,433.,863.,440.,786.,447.,795.,454.,812.,461.,827.,468.,787.,475.,760.,482.,751.,489.,789.};
            
            kinkFitLine(testPntsA,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestB?")) {
            
            cv::Matx<double,102,2> testPntsB{343.,419.,343.,413.,344.,421.,344.,418.,344.,401.,345.,411.,345.,399.,345.,394.,346.,391.,346.,386.,347.,377.,347.,370.,347.,366.,348.,374.,348.,368.,348.,358.,349.,353.,349.,340.,349.,347.,350.,342.,350.,338.,350.,322.,351.,329.,351.,318.,351.,316.,352.,310.,352.,307.,352.,303.,353.,302.,353.,293.,354.,289.,354.,285.,354.,280.,355.,271.,355.,272.,355.,268.,356.,259.,356.,251.,356.,256.,357.,248.,357.,245.,357.,246.,358.,237.,358.,219.,358.,223.,359.,221.,359.,216.,360.,210.,360.,207.,360.,193.,361.,198.,361.,194.,361.,184.,362.,185.,362.,179.,362.,171.,363.,177.,363.,167.,363.,164.,364.,153.,364.,153.,364.,144.,365.,136.,365.,139.,365.,138.,366.,132.,366.,126.,367.,123.,367.,114.,367.,109.,368.,104.,368.,106.,368.,95.,369.,93.,369.,88.,369.,89.,371.,94.,372.,94.,374.,103.,375.,102.,377.,106.,379.,114.,380.,116.,382.,128.,384.,124.,385.,128.,387.,144.,388.,144.,390.,153.,392.,148.,393.,160.,395.,155.,396.,166.,398.,155.,400.,169.,401.,176.,403.,175.,405.,184.,406.,191.,408.,192.,409.,195.,411.,204.};
            
            kinkFitLine(testPntsB,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestC?")) {
            
            cv::Matx<double,102,2> testPntsC{241.,92.,242.,108.,242.,102.,243.,96.,243.,140.,244.,117.,244.,116.,245.,90.,246.,137.,246.,122.,247.,105.,247.,134.,248.,133.,248.,125.,249.,128.,250.,140.,250.,153.,251.,133.,251.,177.,252.,175.,253.,163.,253.,179.,254.,182.,254.,183.,255.,185.,255.,184.,256.,190.,256.,220.,259.,187.,262.,186.,265.,190.,268.,164.,271.,183.,274.,177.,277.,165.,280.,183.,283.,186.,285.,178.,288.,145.,291.,167.,294.,139.,297.,169.,300.,157.,303.,130.,306.,157.,309.,143.,312.,156.,315.,171.,318.,126.,321.,141.,324.,151.,327.,129.,330.,138.,333.,109.,336.,124.,338.,109.,341.,122.,344.,115.,347.,149.,350.,128.,353.,113.,356.,117.,359.,124.,362.,113.,365.,114.,368.,90.,371.,115.,374.,103.,377.,98.,380.,111.,383.,80.,386.,107.,389.,78.,392.,100.,394.,66.,397.,74.,400.,78.,403.,74.,406.,79.,409.,89.,412.,70.,415.,80.,418.,85.,421.,81.,424.,61.,427.,58.,430.,54.,433.,69.,436.,58.,439.,55.,442.,64.,445.,114.,447.,42.,450.,57.,453.,79.,456.,37.,459.,24.,462.,62.,465.,51.,468.,9.,471.,30.,474.,28.};
            
            kinkFitLine(testPntsC,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestD?")) {
            
            cv::Matx<double,102,2> testPntsD{212.,720.,218.,760.,224.,820.,230.,869.,236.,792.,242.,798.,248.,790.,254.,888.,260.,809.,266.,821.,272.,869.,278.,844.,284.,838.,290.,830.,296.,882.,302.,906.,308.,851.,314.,867.,320.,880.,326.,879.,332.,887.,338.,983.,344.,967.,350.,934.,350.,977.,350.,909.,351.,951.,351.,939.,352.,924.,352.,902.,352.,917.,353.,867.,353.,839.,354.,867.,354.,827.,355.,863.,355.,771.,355.,774.,356.,809.,356.,778.,357.,757.,357.,757.,357.,794.,358.,784.,358.,726.,359.,762.,359.,736.,360.,720.,360.,734.,360.,619.,361.,651.,361.,696.,362.,638.,362.,647.,362.,639.,363.,652.,363.,569.,364.,645.,364.,605.,365.,579.,365.,594.,365.,580.,366.,550.,366.,537.,367.,549.,367.,546.,367.,501.,368.,518.,368.,484.,369.,468.,369.,483.,370.,440.,370.,430.,370.,461.,371.,448.,371.,369.,372.,385.,372.,452.,372.,402.,373.,460.,373.,430.,374.,371.,374.,317.,375.,317.,375.,381.,375.,322.,376.,285.,376.,280.,377.,296.,377.,279.,377.,222.,378.,272.,378.,242.,379.,228.,379.,243.,380.,234.,380.,254.,380.,163.,381.,173.,381.,162.,382.,121.,382.,163.};
            
            kinkFitLine(testPntsD,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestE?")) {
            
            cv::Matx<double,102,2> testPntsE{535.,494.,541.,432.,548.,495.,554.,531.,561.,492.,567.,420.,574.,429.,580.,473.,587.,430.,593.,400.,600.,439.,606.,453.,613.,417.,619.,362.,626.,320.,632.,409.,639.,370.,645.,392.,652.,441.,658.,408.,665.,409.,671.,399.,678.,367.,684.,408.,691.,327.,697.,317.,704.,337.,710.,319.,717.,250.,723.,337.,730.,319.,736.,286.,743.,257.,749.,264.,756.,353.,762.,230.,762.,279.,764.,282.,765.,361.,767.,257.,768.,293.,770.,289.,772.,232.,773.,365.,775.,292.,776.,377.,778.,356.,779.,383.,781.,343.,783.,411.,784.,409.,786.,345.,787.,438.,789.,407.,791.,439.,792.,476.,794.,420.,795.,494.,797.,470.,798.,498.,800.,509.,802.,474.,803.,495.,805.,526.,806.,503.,808.,535.,810.,459.,811.,506.,813.,533.,814.,553.,816.,542.,817.,565.,819.,588.,821.,604.,822.,636.,824.,595.,825.,603.,827.,616.,829.,627.,830.,637.,832.,699.,833.,667.,835.,632.,836.,649.,838.,677.,840.,728.,841.,686.,843.,709.,844.,732.,846.,750.,848.,752.,849.,753.,851.,804.,852.,764.,854.,812.,855.,821.,857.,781.,859.,828.,860.,855.,862.,852.,863.,806.,865.,827.};
            
            kinkFitLine(testPntsE,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestF?")) {
            
            cv::Matx<double,102,2> testPntsF{128.,858.,135.,800.,142.,769.,149.,854.,156.,823.,162.,821.,169.,824.,176.,835.,183.,862.,190.,783.,197.,738.,204.,727.,211.,715.,217.,732.,224.,713.,231.,720.,238.,684.,245.,758.,252.,736.,259.,702.,266.,757.,272.,684.,279.,723.,286.,704.,293.,701.,300.,632.,307.,654.,314.,720.,321.,658.,327.,724.,334.,620.,341.,729.,348.,633.,355.,592.,362.,558.,369.,556.,376.,608.,382.,614.,389.,605.,396.,628.,403.,576.,410.,585.,410.,633.,418.,589.,426.,610.,434.,612.,442.,597.,449.,648.,457.,649.,465.,651.,473.,585.,481.,623.,489.,621.,497.,624.,505.,647.,513.,642.,521.,581.,528.,704.,536.,674.,544.,660.,552.,777.,560.,645.,568.,640.,576.,741.,584.,711.,592.,714.,600.,653.,607.,768.,615.,703.,623.,682.,631.,778.,639.,686.,647.,736.,655.,740.,663.,749.,671.,710.,679.,716.,686.,672.,694.,811.,702.,785.,710.,754.,718.,766.,726.,707.,734.,746.,742.,734.,750.,701.,758.,783.,765.,739.,773.,694.,781.,770.,789.,780.,797.,749.,805.,715.,813.,785.,821.,777.,829.,808.,837.,824.,844.,851.,852.,858.,860.,759.,868.,812.,876.,770.};
            
            kinkFitLine(testPntsF,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestG?")) {
            
            cv::Matx<double,102,2> testPntsG{47.,958.,48.,943.,50.,927.,51.,906.,52.,924.,54.,863.,55.,861.,56.,836.,58.,817.,59.,808.,60.,793.,62.,778.,63.,746.,64.,691.,66.,690.,67.,751.,69.,703.,70.,651.,71.,638.,73.,641.,74.,614.,75.,592.,77.,601.,78.,550.,79.,564.,81.,536.,82.,462.,83.,495.,85.,447.,86.,451.,87.,425.,89.,388.,90.,383.,91.,347.,93.,330.,94.,311.,95.,318.,97.,278.,98.,290.,100.,264.,101.,259.,102.,254.,104.,197.,105.,197.,106.,197.,108.,132.,109.,92.,110.,75.,112.,124.,113.,74.,114.,99.,116.,38.,117.,30.,117.,38.,123.,43.,128.,70.,134.,119.,139.,94.,145.,127.,150.,140.,156.,182.,161.,166.,167.,179.,172.,194.,178.,236.,184.,217.,189.,247.,195.,314.,200.,286.,206.,299.,211.,305.,217.,331.,222.,341.,228.,388.,233.,404.,239.,386.,244.,433.,250.,440.,256.,459.,261.,482.,267.,481.,272.,510.,278.,518.,283.,560.,289.,533.,294.,600.,300.,606.,305.,612.,311.,625.,316.,643.,322.,659.,328.,686.,333.,732.,339.,705.,344.,710.,350.,739.,355.,747.,361.,789.,366.,801.,372.,820.,377.,813.,383.,820.};
            
            kinkFitLine(testPntsG,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestH?")) {
            
            cv::Matx<double,102,2> testPntsH{470.,89.,472.,102.,473.,111.,475.,127.,476.,135.,478.,153.,479.,148.,481.,158.,482.,144.,484.,156.,485.,172.,487.,186.,489.,183.,490.,188.,492.,222.,493.,217.,495.,217.,496.,221.,498.,236.,499.,258.,501.,237.,502.,262.,504.,257.,504.,276.,509.,273.,515.,287.,520.,295.,526.,282.,531.,285.,537.,267.,542.,296.,548.,319.,553.,328.,559.,296.,564.,308.,570.,318.,575.,349.,581.,352.,586.,335.,592.,338.,597.,379.,603.,345.,608.,373.,613.,367.,619.,381.,624.,367.,630.,363.,635.,394.,641.,369.,646.,408.,652.,403.,657.,425.,663.,410.,668.,436.,674.,419.,679.,421.,685.,443.,690.,434.,696.,465.,701.,457.,707.,452.,712.,470.,718.,478.,723.,488.,728.,475.,734.,487.,739.,509.,745.,494.,750.,495.,756.,524.,761.,520.,767.,538.,772.,507.,778.,548.,783.,535.,789.,531.,794.,534.,800.,560.,805.,548.,811.,574.,816.,584.,822.,546.,827.,560.,832.,577.,838.,581.,843.,594.,849.,589.,854.,615.,860.,621.,865.,596.,871.,588.,876.,618.,882.,608.,887.,655.,893.,668.,898.,641.,904.,665.,909.,666.,915.,660.,920.,667.,926.,669.,931.,703.};
            
            kinkFitLine(testPntsH,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestI?")) {
            
            cv::Matx<double,102,2> testPntsI{433.,899.,434.,890.,435.,886.,436.,866.,437.,869.,439.,856.,440.,846.,441.,846.,442.,835.,443.,830.,444.,815.,445.,810.,446.,796.,447.,795.,448.,783.,450.,774.,451.,768.,452.,765.,453.,754.,454.,745.,455.,741.,456.,732.,457.,718.,458.,706.,459.,702.,461.,696.,462.,685.,463.,692.,464.,681.,465.,663.,466.,657.,467.,654.,468.,632.,469.,640.,470.,624.,472.,612.,473.,603.,474.,601.,475.,594.,476.,585.,477.,576.,478.,568.,479.,561.,480.,554.,481.,542.,483.,531.,484.,517.,485.,517.,486.,511.,487.,504.,488.,490.,489.,489.,490.,478.,491.,466.,492.,464.,494.,449.,495.,442.,496.,440.,497.,429.,498.,417.,499.,420.,500.,402.,501.,397.,502.,396.,503.,385.,505.,378.,506.,363.,507.,356.,508.,351.,509.,344.,510.,325.,511.,323.,512.,316.,513.,310.,514.,302.,516.,290.,517.,285.,518.,276.,519.,266.,520.,260.,520.,262.,527.,264.,535.,258.,542.,251.,550.,259.,557.,245.,564.,236.,572.,239.,579.,236.,586.,225.,594.,223.,601.,222.,609.,212.,616.,221.,623.,208.,631.,210.,638.,208.,645.,207.,653.,200.,660.,194.,668.,195.,675.,191.};
            
            kinkFitLine(testPntsI,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestJ?")) {
            
            cv::Matx<double,102,2> testPntsJ{441.,30.,446.,88.,450.,137.,455.,127.,459.,49.,464.,64.,468.,144.,473.,159.,478.,166.,482.,119.,487.,170.,491.,114.,496.,115.,500.,200.,505.,239.,509.,192.,514.,216.,519.,193.,523.,241.,528.,235.,532.,261.,537.,249.,541.,287.,546.,380.,551.,339.,555.,306.,560.,317.,564.,299.,569.,393.,573.,332.,578.,357.,583.,384.,587.,308.,592.,417.,596.,394.,601.,402.,605.,431.,610.,385.,615.,438.,619.,429.,624.,457.,628.,512.,633.,577.,637.,478.,642.,529.,646.,453.,651.,542.,656.,506.,660.,502.,665.,603.,669.,558.,674.,631.,678.,557.,683.,600.,688.,614.,692.,647.,697.,711.,701.,618.,706.,652.,710.,694.,715.,703.,720.,740.,724.,720.,729.,732.,733.,679.,738.,696.,742.,723.,747.,737.,752.,768.,756.,752.,761.,820.,765.,884.,770.,735.,774.,787.,779.,796.,783.,881.,788.,913.,793.,868.,797.,935.,802.,858.,806.,835.,811.,888.,815.,929.,820.,955.,820.,867.,828.,919.,837.,847.,845.,947.,853.,827.,861.,845.,870.,852.,878.,885.,886.,849.,895.,854.,903.,833.,911.,867.,920.,886.,928.,871.,936.,832.,944.,815.,953.,830.,961.,741.};
            
            kinkFitLine(testPntsJ,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestK?")) {
            
            cv::Matx<double,102,2> testPntsK{361.,367.,365.,380.,369.,348.,373.,358.,377.,381.,381.,399.,385.,377.,389.,358.,394.,376.,398.,366.,402.,424.,406.,374.,410.,386.,414.,364.,418.,355.,422.,402.,426.,352.,430.,356.,434.,365.,438.,347.,442.,362.,446.,329.,451.,322.,455.,333.,459.,337.,463.,319.,467.,377.,471.,319.,475.,326.,479.,266.,483.,351.,487.,301.,491.,312.,495.,250.,499.,297.,503.,283.,508.,319.,512.,280.,516.,359.,520.,310.,524.,330.,528.,295.,532.,292.,536.,296.,540.,265.,544.,277.,548.,309.,552.,357.,556.,265.,560.,321.,564.,272.,569.,251.,573.,243.,577.,250.,581.,232.,585.,263.,589.,245.,593.,283.,597.,226.,601.,258.,605.,308.,609.,197.,613.,257.,617.,196.,621.,264.,626.,286.,630.,232.,634.,250.,638.,234.,642.,211.,646.,254.,650.,244.,654.,291.,654.,179.,657.,237.,661.,251.,664.,185.,667.,170.,671.,217.,674.,208.,678.,188.,681.,195.,684.,210.,688.,192.,691.,164.,694.,229.,698.,176.,701.,174.,704.,129.,708.,188.,711.,192.,714.,187.,718.,143.,721.,140.,724.,98.,728.,120.,731.,178.,735.,167.,738.,118.,741.,123.,745.,136.,748.,138.};
            
            kinkFitLine(testPntsK,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        if (yesno("Run KinkFit TestL?")) {
            
            cv::Matx<double,102,2> testPntsL{478.,233.,483.,280.,488.,274.,493.,253.,498.,251.,503.,171.,508.,243.,513.,173.,518.,209.,523.,195.,528.,200.,533.,158.,539.,222.,544.,139.,549.,158.,554.,197.,559.,138.,564.,101.,569.,55.,574.,76.,579.,113.,584.,112.,589.,28.,594.,108.,594.,52.,597.,64.,600.,78.,603.,90.,606.,166.,610.,135.,613.,152.,616.,99.,619.,132.,622.,180.,625.,131.,628.,184.,631.,179.,634.,174.,637.,195.,641.,179.,644.,200.,647.,172.,650.,222.,653.,225.,656.,201.,659.,266.,662.,269.,665.,262.,668.,273.,672.,328.,675.,286.,678.,298.,681.,322.,684.,310.,687.,288.,690.,387.,693.,354.,696.,329.,700.,389.,703.,355.,706.,367.,709.,410.,712.,409.,715.,403.,718.,399.,721.,455.,724.,438.,727.,519.,731.,507.,734.,499.,737.,520.,740.,489.,743.,510.,746.,522.,749.,472.,752.,464.,755.,547.,759.,547.,762.,565.,765.,588.,768.,589.,771.,568.,774.,642.,777.,609.,780.,595.,783.,597.,786.,616.,790.,630.,793.,665.,796.,640.,799.,662.,802.,648.,805.,671.,808.,673.,811.,730.,814.,676.,817.,703.,821.,758.,824.,729.,827.,724.,830.,755.,833.,787.};
            
            kinkFitLine(testPntsL,line,CV_DIST_L2,0,0.01,0.01);
            
        };
        
        
        
    };

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
        
    }
    
    // Run ColorSpace Test
    if (yesno("Run ColorSpace Test?")) {
        
        Mat rgbImg(Size(256,256),CV_8UC3);
        for (int r=0; r < rgbImg.rows; r++) {
            for (int g=0; g < rgbImg.cols; g++) {
                char b=255-ceil((r+g)/2.0) ;
                Vec3b rgbColor{r,g,b};
                rgbImg.at<Vec3b>(Point(r,g)) = rgbColor;
            }
        }
        printImg<CV_8UC3>(rgbImg,"rgbImgTest");
        
        RGB2Rot<CV_8UC3,CV_8UC3> rot;
        rot = *new RGB2Rot<CV_8UC3,CV_8UC3>();
        
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0261007, 0.0115076};
        cv::Vec<double, 3> uG{-0.235702, -27.0915, -61.4471};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};
        rot = *new RGB2Rot<CV_8UC3,CV_8UC3>(2, 2, theta, uG, uC);
       // RGB2Rot<CV_8UC3,CV_8UC3> rot(2, 2, theta, uG, uC);
        Mat LCaCbImg(rgbImg.size(),CV_8UC3);
        cv::convertColor<CV_8UC3,CV_8UC3>(rgbImg, LCaCbImg, rot);
        
        printImg<CV_8UC3>(LCaCbImg,"LCaCbImg");
        
        imshow("source", rgbImg);
        namedWindow("result", 1);
        imshow("result", LCaCbImg);
        
        // Wait for a key stroke; the same function arranges events processing
        waitKey();
        

    }
    
    if (yesno("Run Shape Tests?")) {
        double a=2.0,  b =3.0;
        cv::Point2f center(100.,200.);
        double angle = 0.2;
        bool radians = true;
        Ellipse ellipse(a,b,center,angle,radians);
        ellipse.setRadians(true);
        ellipse.print();
        
        a=2.0;  b =3.0;
        center.x = 100.; center.y = 200.;
        angle = 20;
        radians = false;
        ellipse = Ellipse(a,b,center,angle,radians);
        ellipse.print();
        ellipse.setRadians(true);
        ellipse.print();
        
        graveShape tip(Point2f(192.928711, 221.524353), Point2f(194.835342, 123.243004),
                       Point2f(70.960808, 209.591858),Point2f(88.138336, 112.804512),
                       Ellipse(52.424313, 72.482826, Point2f(142.548141, 129.361160),  6.282559));
        tip.print("tip");
        
        tip = graveShape(Point2f(56.318989, 288.310059), Point2f(107.110855, 204.149246),
                         Point2f(-43.342068, 216.992279),Point2f(19.927784, 141.760742),
                       Ellipse(52.424313, 72.482826, Point2f(58.769733, 183.304123),  0.522972));
        tip.print("tip");
        int maxRot=24;
        for(int i=0; i<maxRot;i++){
            tip.rotate(2*CV_PI/maxRot);
            tip.print("tip");
        }
    };
    
    // Run FingerModel Test
    if (yesno("Run FingerModel Test?")) {
        
        cv::CommandLineParser parser(argc, argv, "{help h||}{@image|../imgJMiddle2.JPG|}");
        if (parser.has("help"))
        {
            help();
            return 0;
        }
        string filename = parser.get<string>("@image");
        image = imread(filename, IMREAD_COLOR );
        if( image.empty() )
        {
            cout << "Couldn't open image " << filename << "\n";
            return 0;
        }
        
        printImg<CV_8UC3>(image,"image");
        
        // Setup color Space
        double theta = 0.902576829326826;
        cv::Vec<double, 3> uS{3., 0.0600316, 0.0264675};
        cv::Vec<double, 3> uG{-0.235702, -11.7789, -26.7161};
        cv::Vec<double, 3> uC{0.5, 0.356556, 0.478808};
        
        RGB2Rot<CV_8UC3,CV_8UC4> rot;
        rot = *new RGB2Rot<CV_8UC3,CV_8UC4>();
        rot = *new RGB2Rot<CV_8UC3,CV_8UC4>(0, 2, theta, uG, uC);
        
      //  RGB2Rot<CV_8UC3,CV_8UC4> rot(0, 2, theta, uG, uC);
        
        // Convert image to LCaCb
        Mat LCaCbImg(image.size(),CV_8UC4);
        cv::convertColor<CV_8UC3,CV_8UC4>(image, LCaCbImg, rot);
        
        printImg<CV_8UC4>(LCaCbImg,"LCaCbImg");
        
        Point midPnt, perpVec;
        findFrameOrientation<CV_8UC4>(LCaCbImg, &midPnt, &perpVec);
        
        // Find the tip point.
        Point tipPnt;
        tipPnt = runReachToEnd<CV_8UC4>(LCaCbImg, midPnt, perpVec);
        
        // fillamentFill
        Mat edgePnts, midPnts;
        fillamentFill<CV_8UC4>(LCaCbImg, edgePnts, midPnts, midPnt, tipPnt);
        
        printImg<CV_32SC1>(edgePnts,"edgePnts");
        printImg<CV_32SC1>(midPnts,"midPnts");
        
        // Exclude Points
        int count = midPnts.rows;
        int rows = LCaCbImg.rows, cols = LCaCbImg.cols;
        
        Mat edgePntsClass(edgePnts.rows,  1, CV_8U);
        edgePntsClass= Mat::zeros(edgePnts.rows,  1, CV_8U);
        
        Mat  midPntsClass(edgePnts.rows/2,1, CV_8U);
        midPntsClass= Mat::zeros(edgePnts.rows/2,  1, CV_8U);
        
        int tipCount = 0, distalCount = 0;
        
       int frameCount = classifyPointOnFrame(LCaCbImg, edgePnts, midPnts, edgePntsClass, midPntsClass);
        
        classifyPointInitial(perpVec, edgePnts, midPnts, edgePntsClass, midPntsClass, tipCount, distalCount);
            
        printImg<CV_8U>(edgePntsClass,"edgePntsClass");
        printImg<CV_8U>(midPntsClass,"midPntsClass");
        
        Mat classImg(LCaCbImg.size(),CV_8UC3);
        classImg = Mat::zeros(LCaCbImg.size(),CV_8UC3);
        for (int i=0; i<edgePntsClass.rows; i++) {
            Point pnt = edgePnts.at<Point>(i,0);
            pointClass cls = edgePntsClass.at<pointClass>(i,0);
            Vec3b color = Vec3b(0,0,0);
            switch (cls) { // unclassified, tip, distal, middle, onFrame, anomaly
                case unclassified:
                    color = Vec3b(255,255,255);
                    break;
                case tip:
                    color = Vec3b(255,0,255);
                    break;
                case distal:
                    color = Vec3b(255,255,0);
                    break;
                case middle:
                    color = Vec3b(0,255,255);
                    break;
                case onFrame:
                    color = Vec3b(255,0,0);
                    break;
                case anomaly:
                    color = Vec3b(128,0,0);
                    break;
                default:
                    color = Vec3b(0,0,0);
                    break;
            }
            classImg.at<Vec3b>(pnt) = color;
        }
        
        Mat distalMidPnts = extractPntsClassifiedAs(distal, midPnts, midPntsClass);

        printImg<CV_32SC1>(distalMidPnts,"distalMidPnts");
        
        cv::Matx<double,3,2> line;
        kinkFitLine(distalMidPnts, line,CV_DIST_L2,0,0.01,0.01);
        printImg<CV_64FC1>(line,"line");
        
        Mat distalEdgePnts = extractPntsClassifiedAs(distal, edgePnts, edgePntsClass);
        cv::Matx<double,2,2> endLine{line(1,0),line(1,1),line(2,0),line(2,1)};
        Mat trapFit = trapeziumFit(endLine, distalEdgePnts);
        
        
        Mat distalTip = extractPntsClassifiedAs(tip, edgePnts, edgePntsClass);
        
        RotatedRect tipEllipse = fitEllipseDirect(distalTip);
        Ellipse ellipse(tipEllipse);
        ellipse.setRadians(true);
        Point2f tipPoint = ellipse.intersectionWithLine(Point2f(line(1,0),line(1,1)),Point2f(line(2,0),line(2,1)));
        fprintf(stdout,"line = {{%f, %f},{%f, %f}}\n",line(1,0),line(1,1),tipPoint.x,tipPoint.y);
        ellipse.print();
        
        fingerTipModel mdl(Point2f(trapFit.at<float>(0,0),trapFit.at<float>(0,1)),
                           Point2f(trapFit.at<float>(1,0),trapFit.at<float>(1,1)),
                           Point2f(trapFit.at<float>(2,0),trapFit.at<float>(2,1)),
                           Point2f(trapFit.at<float>(3,0),trapFit.at<float>(3,1)), ellipse);
        mdl.print();
        mdl.mdl.alignCurveToTrapezium();
        mdl.print();
        mdl.alignToEllipseCenter();
        mdl.print();
        mdl.alignToTrapezium();
        mdl.print();
        
        Mat mdlPnts = mdl.imgMdl.pointsOnLeft();
        printImg<CV_32F>(mdlPnts,"mdlPnts");
        
        Mat arcPnts;
        
        fprintf (stdout, "%s"," ellipsePnts={");
        arcPnts = mdl.mdl.curve.pointsOnArc(mdl.mdl.theta0, mdl.mdl.theta1, 4);
        printImg<CV_32F>(arcPnts,"ellipse5Pnts");
        
        fprintf (stdout, "%s"," ellipsePnts={");
        arcPnts = mdl.mdl.curve.pointsOnArc(mdl.mdl.theta0, mdl.mdl.theta1, 100);
        printImg<CV_32F>(arcPnts,"ellipsePnts");
        
        
        
        printImg<CV_32S>(distalTip,"distalTipN");
        
        
        
        
        
        imshow("source", image);
        namedWindow("result", 1);
        imshow("result", LCaCbImg);
        
        // Wait for a key stroke; the same function arranges events processing
        waitKey();
        
    
        }
    // ****************************************************
    
    
    
    // Run Elliptical Fit Test
    if (yesno("Run Elliptical Fit Test?")) {
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

    }
    
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

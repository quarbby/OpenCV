//Testing blob2.cpp

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// For red image
int lowerH=10, lowerS=0, lowerV=0;
int upperH=79, upperS=148, upperV=255;

// For yellow image
// int lowerH=0, lowerS=0, lowerV=100;
// int upperH=77, upperS=195, upperV=251;

//Structure for bounding box
struct RectData {
	bool detected;
	double heading, angle;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

cv::Mat image;

using namespace cv;


void drawImage(){
	cv::Mat red_image;
	cv::cvtColor(image, red_image, CV_BGR2HSV);
	cv::inRange(image, cv::Scalar(lowerH,lowerS,lowerV), cv::Scalar(upperH,upperS,upperV), red_image);
	cv::Mat erodeEl = cv::getStructuringElement(MORPH_RECT, cv::Size(9, 9));
	cv::Mat dilateEl = cv::getStructuringElement(MORPH_RECT, cv::Point(7, 7));
	cv::erode(red_image, red_image, erodeEl, Point(-1, -1), 1);
	cv::dilate(red_image, red_image, dilateEl, Point(-1, -1), 1);
	cv::imshow("blob-out", red_image);
}

void lowerHCallback(int val, void *params){
	lowerH = val;
	drawImage();
}

void upperHCallback(int val, void *params){
	upperH = val;
	drawImage();
}

void lowerSCallback(int val, void *params){
	lowerS = val;
	drawImage();
}

void upperSCallback(int val, void *params){
	upperS = val;
	drawImage();
}

void lowerVCallback(int val, void *params){
	lowerV = val;
	drawImage();
}

void upperVCallback(int val, void *params){
	upperV = val;
	drawImage();
}

void setWindowSettings(){
	cv::namedWindow("blob-in");

	cv::namedWindow("blob-out");
	cv::namedWindow("trackbar");

	cv::createTrackbar("LowerH", "trackbar", &lowerH, 180, lowerHCallback, NULL);
    cv::createTrackbar("UpperH", "trackbar", &upperH, 180, upperHCallback, NULL);

	cv::createTrackbar("LowerS", "trackbar", &lowerS, 256, lowerSCallback, NULL);
    cv::createTrackbar("UpperS", "trackbar", &upperS, 256, upperSCallback, NULL);

	cv::createTrackbar("LowerV", "trackbar", &lowerV, 256, lowerVCallback, NULL);
    cv::createTrackbar("UpperV", "trackbar", &upperV, 256, upperVCallback, NULL); 
}

int main(int argc, char** argv){
	image = cv::imread(argv[1]);
	RectData rectData;
	cv::resize(image, image, cv::Size(600,480));
	
	setWindowSettings();
	drawImage();
	cv::imshow("blob-in", image);


	cv::waitKey();
	return (0);
}

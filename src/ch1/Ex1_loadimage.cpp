//Loads image from disk
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
	IplImage* img = cvLoadImage(argv[1]);
	cvNamedWindow("Ex 1: Flare", CV_WINDOW_AUTOSIZE);
	cvShowImage("Flare", img);
	cvWaitKey(0);
	cvReleaseImage(&img);
	cvDestroyWindow("Flare");
}
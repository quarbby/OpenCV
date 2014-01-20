//Ex 4: Image with smoothing 
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

void example2_4(IplImage* image){
	//Create some windows to show input and output images
	cvNamedWindow("Example4-in");
	cvNamedWindow("Example4-out");

	cvShowImage("Example4-in", image);

	//Create image to hold smoothed output
	IplImage* out = cvCreateImage(
		cvGetSize(image),
		IPL_DEPTH_8U, 
		3
	);
	//Do the smoothing 
	cvSmooth(image, out, CV_GAUSSIAN, 3, 3);
	//Display smoothed image
	cvShowImage("Example4-out", out);
	cvReleaseImage(&out);

	//Wait for user to hit key then cleanup windows
	cvWaitKey(0);
	cvDestroyWindow("Example4-in");
	cvDestroyWindow("Example4-out");
}

int main(int argc, char** argv){
	IplImage* img = cvLoadImage("tiffany.jpg");
	example2_4(img);

}
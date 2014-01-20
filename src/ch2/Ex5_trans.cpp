// Sequence of transformations
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

IplImage* doPyrDown( IplImage* in, int filter = IPL_GAUSSIAN_5x5 ) {
	//Best to make sure input image is divisible by 2
	assert(in->width%2 == 0 && in->height%2 == 0);

	IplImage* out = cvCreateImage(
		cvSize(in->width/2, in->height/2),
		in->depth, 
		in->nChannels
	);
	cvPyrDown(in, out);
	return (out);
};

int main(int argc, char** argv){
	IplImage *img = cvLoadImage("tiffany.jpg");
	cvNamedWindow("Example5-in");
	cvShowImage("Example5-in", img);

	IplImage *out = doPyrDown(img);
	cvNamedWindow("Example5");
	cvShowImage("Example5", out);
	cvWaitKey(0);
	cvDestroyWindow("Example5"); 
	cvDestroyWindow("Example5-in");
}
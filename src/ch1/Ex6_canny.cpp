//A canny edge detector writes output to single channel grayscale image
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

IplImage* doCanny( 	IplImage* in, double lowThres, double highThres, double aperture ) {
	if(in->nChannels != 1) return 0;	// Canny handles only gray scale images 

	IplImage* out = cvCreateImage(cvSize (in->width, in->height), IPL_DEPTH_8U, 1);
	cvCanny(in, out, lowThres, highThres, aperture);
	return (out);
};

int main (int argc, char** argv){
	IplImage *img = cvLoadImage("disney.jpg");
	cvNamedWindow("Example6-in");
	cvShowImage("Example6-in", img);

	IplImage *out = doCanny(img, 10.0, 100.0, 3.0);
	cvNamedWindow("Example6-out");
	cvShowImage("Example6-out", out);

	cvWaitKey(0);
	cvDestroyWindow("Example5-in");
	cvDestroyWindow("Example6-out");
}

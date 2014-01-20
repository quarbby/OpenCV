//Using ImageROI to increment all the pixels in a region
//roi_add <image> <x> <y> <width> <height <add>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){
	IplImage* src;
	if (argc == 7 && ((src=cvLoadImage(argv[1], 1)) != 0)){
		int x = atoi(argv[2]);
		int y = atoi(argv[3]);
		int width = atoi(argv[4]);
		int height = atoi(argv[5]);

		int add = atoi(argv[6]);
		cvSetImageROI(src, cvRect(x, y, width, height));
		cvAddS(src, cvScalar(add), src);
		cvResetImageROI(src);
		cvNamedWindow("Roi_add", 1);
		cvShowImage("Roi_add", src);
		cvWaitKey();
	}
	return 0;
}
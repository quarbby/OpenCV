//Using widthStep method to incremenet all pixels of interest image by 1
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){
	IplImage *interest_img = cvLoadImage(argv[1]);
	CvRect interest_rect = cvRect (10, 10, 120, 120);

	IplImage *sub_img = cvCreateImageHeader(
		cvSize(interest_rect.width, interest_rect.height),
		interest_img->depth, interest_img->nChannels
	);
	sub_img->origin = interest_img->origin;
	sub_img->widthStep = interest_img->widthStep;
	sub_img->imageData = interest_img->imageData + interest_rect.y*interest_img->widthStep +
						interest_rect.x * interest_img->nChannels;

	cvAddS(sub_img, cvScalar(1), sub_img);

	cvNamedWindow("Widthstep_add", 1);
	cvShowImage("Widthstep_add", sub_img);
	cvWaitKey();
	cvReleaseImageHeader(&sub_img);


	return (0);
}
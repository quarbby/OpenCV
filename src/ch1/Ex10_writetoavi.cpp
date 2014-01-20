//Read in a colour video and write out the same video in grayscale
//argv[1] : input video file
//argv[2] : output file namespace

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv){

	CvCapture* capture = 0;
	capture = cvCreateFileCapture(argv[1]);
	if (!capture){
		return -1;
	}

	IplImage *bgr_frame = cvQueryFrame(capture); 	//Init the video read
	double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	CvSize size = cvSize ((int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH), 
							(int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT));
	CvVideoWriter *writer = cvCreateVideoWriter(argv[2], CV_FOURCC('M', 'J', 'P', 'G'), fps, size);

	IplImage* logpolar_frame = cvCreateImage(size, IPL_DEPTH_8U, 3);

	while ( (bgr_frame=cvQueryFrame(capture)) != NULL){
		cvLogPolar(bgr_frame, logpolar_frame, 
			cvPoint2D32f(bgr_frame->width/2, bgr_frame->height/2), 40,
			CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
		cvWriteFrame(writer, logpolar_frame);
	}
	cvReleaseVideoWriter(&writer);
	cvReleaseImage(&logpolar_frame);
	cvReleaseCapture(&capture);

	cvNamedWindow("Example10", CV_WINDOW_AUTOSIZE);
	capture = cvCreateFileCapture(argv[2]);
	IplImage* frame;
	while (1){
		frame = cvQueryFrame(capture);
		if(!frame) break;
		cvShowImage("Example10", frame);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("Example10");

	return (0);
}
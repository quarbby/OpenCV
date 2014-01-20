//Using a mouse to draw boxes on the screen
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

//Define callback to install for mouse events
void my_mouse_callback(int event, int x, int y, int flags, void* param);

CvRect box;
bool drawing_box = false;

//Subroutine to draw box onto an image
void draw_box(IplImage *img, CvRect rect){
	cvRectangle(img, cvPoint(box.x, box.y), 
		cvPoint(box.x+box.width, box.y+box.height),
		cvScalar(0xff, 0x00, 0x00) //red
	);
}

int main (int argc, char** argv){
	box = cvRect(-1,-1,0,0);

	IplImage *image = cvCreateImage(cvSize(500,500), IPL_DEPTH_8U, 3);
	cvZero(image);
	IplImage *temp = cvCloneImage(image);

	cvNamedWindow("Box Example");

	//Install the callback
	cvSetMouseCallback("Box Example", my_mouse_callback, (void*) image);

	//While the user is drawing, put the currently contemplated box onto the temp image
	//Display the temp image and wait 15ms for a keystroke, then repeat
	while(1){
		cvCopyImage(image, temp);
		if(drawing_box) draw_box(temp, box);
		cvShowImage("Box Example", temp);
		if (cvWaitKey(15) == 27) break;
	}

	//Clean up
	cvReleaseImage(&image);
	cvReleaseImage(&temp);
	cvDestroyWindow("Box Example");
}

//Mouse callback function
void my_mouse_callback(int event, int x, int y, int flags, void* param){
	IplImage *image = (IplImage*) param;

	switch(event){
		case CV_EVENT_MOUSEMOVE: 
		{
			if (drawing_box){
				box.width = x-box.x;
				box.height = y-box.y;
			}
		}
		break;

		case CV_EVENT_LBUTTONDOWN:
		{
			drawing_box = true;
			box = cvRect(x, y, 0, 0);
		}

		break;

		case CV_EVENT_LBUTTONUP:
		{
			drawing_box = false;
			if (box.width < 0){
				box.x += box.width;
				box.width *= -1;
			}
			if (box.height < 0){
				box.y += box.height;
				box.height *= -1;
			}
			draw_box;
		}
		break;
	}
}
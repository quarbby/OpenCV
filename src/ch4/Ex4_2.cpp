//Using a trackbar to create a switch the user can turn on and off
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

int g_switch_value = 0;

//Trackbar callback
void switch_on_function(){

}

void switch_off_function(){
	
}

void switch_callback(int position){
	if (position == 0){
		switch_off_function();
	}
	else {
		switch_on_function();
	}
}

int main(int argc, char** argv){
	cvNamedWindow("Demo Window", 1);

	//Create the trackbar
	cvCreateTrackbar("Switch", "Demo Window", &g_switch_value, 1, switch_callback);

	while(1){
		if (cvWaitKey(15) == 27) break;
	}

	return 0;
}
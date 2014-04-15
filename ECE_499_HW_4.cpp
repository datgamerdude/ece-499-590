//http://stackoverflow.com/questions/1585535/convert-rgb-to-black-white-in-opencv
//
// Some code from here was used to understand how converting to black and white works
//
//http://opencv-srf.blogspot.com/2011/09/capturing-images-videos.html
//
// Code used as the base for getting camera running. Modified off of it

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the video camera no. 0
    //VideoCapture cap("Auto Sprinkler.mp4"); // Video file

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    namedWindow("Source",CV_WINDOW_AUTOSIZE);
    namedWindow("Gray",CV_WINDOW_AUTOSIZE);
    namedWindow("B/W",CV_WINDOW_AUTOSIZE);
    namedWindow("Resize",CV_WINDOW_AUTOSIZE);
//    namedWindow("Blur",CV_WINDOW_AUTOSIZE);
//    namedWindow("HSV",CV_WINDOW_AUTOSIZE);
//    namedWindow("HLS",CV_WINDOW_AUTOSIZE);
//    namedWindow("Canny",CV_WINDOW_AUTOSIZE);
//    namedWindow("Erode",CV_WINDOW_AUTOSIZE);
//    namedWindow("Dilate",CV_WINDOW_AUTOSIZE);

    while (1){
        Mat frame;
	Mat frame_gray;
	Mat frame_bw;
	Mat frame_resize;
	Mat frame_resize_color;
	Mat frame_blur;
	Mat frame_HSV;
	Mat frame_HLS;
	Mat frame_Canny;
	Mat frame_erode;
	Mat frame_dilate;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess){ //if not success, break loop
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

	cvtColor(frame, frame_gray, CV_RGB2GRAY);
	frame_bw = frame_gray > 45;

	resize(frame_bw, frame_resize, Size(), 2, 2, INTER_LINEAR);

	blur(frame_resize, frame_blur, Size(13,13));
	
        imshow("Source", frame); //show the frame in "MyVideo" window
	imshow("Gray", frame_gray);
	imshow("B/W", frame_bw);
	imshow("Resize", frame_resize);
	imshow("Blur", frame_blur);

	resize(frame, frame_resize_color, Size(), 2, 2, INTER_LINEAR);

	cvtColor(frame_resize_color, frame_HSV, CV_RGB2HSV);
	cvtColor(frame_resize_color, frame_HLS, CV_RGB2HLS);

	imshow("HSV", frame_HSV);
	imshow("HLS", frame_HLS);

	blur(frame_resize, frame_blur, Size(3,3));
	Canny(frame_blur,frame_Canny, 100, 300, 3);

	erode(frame_blur, frame_erode, Mat());
	dilate(frame_blur, frame_dilate, Mat());
	
	imshow("Canny", frame_Canny);
	imshow("Erode", frame_erode);
	imshow("Dilate", frame_dilate);

        if (waitKey(30) == 27){ //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
    return 0;

}

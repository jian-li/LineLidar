#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
		//video capture
		VideoCapture video_cap(1);

	 if(!video_cap.isOpened())  // check if we succeeded
        return -1;
	
  namedWindow("image_raw",1);
  Mat gray_image;
  Mat image_raw;

  for(;;)
  {
       	
        video_cap >> image_raw; // get a new frame from camera
        cvtColor(image_raw, gray_image, CV_BGR2GRAY);
        // GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);       
        imshow("image_raw", image_raw);
        if(waitKey(30) >= 0) break;
        //setting the contrast and light
  } 
	return 0;
}

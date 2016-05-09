#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

float alpha_value = 0;
int beta_value = 0;
int alpha_trackerbar_slider = 206;
int beta_trackbar_slider;
const int alpha_value_max = 255, beta_value_max = 255;
const int half_gray_value = 128;

void alpha_trackerbar_callback(int ,void *)
{
    alpha_value = 1.0 * alpha_trackerbar_slider / alpha_value_max;
    // std::cout << "alpha value" << alpha_value << std::endl;
}
  
void beta_trackbar_callback(int, void*)
{
    beta_value = beta_trackbar_slider;
    // std::cout << "alpha value" << beta_value<< std::endl;
}

int main()
{
    //video capture
    VideoCapture video_cap(1);

    // if(!video_cap.isOpened())  // check if we succeeded
        // return -1;

    namedWindow("parameter_setting",1);
    Mat gray_image;
    Mat image_raw ;

    createTrackbar("alpha", "parameter_setting", &alpha_trackerbar_slider, alpha_value_max, alpha_trackerbar_callback);
    createTrackbar("beta", "parameter_setting", &beta_trackbar_slider, beta_value_max, beta_trackbar_callback);

    for(;;)
    {
        video_cap >> image_raw; // get a new frame from camera
        cvtColor(image_raw, gray_image, CV_BGR2GRAY);

        imshow("image_raw", image_raw);
        if(waitKey(30) >= 0) break;
        
        //adjust the brightness and contrast
        Mat constrast_brightness_adjusted_image = gray_image.clone();
        int image_width, image_height;
        image_height = image_raw.size().height;
        image_width = image_raw.size().width;
      
        for( int y = 0; y < gray_image.rows; y++ )
        {
            for( int x = 0; x < gray_image.cols; x++ )
            {
                if(gray_image.at<uchar>(y,x) < 2 * alpha_value * half_gray_value)
                {
                    constrast_brightness_adjusted_image.at<uchar>(y,x) = 0;
                }
                else
                {
                    constrast_brightness_adjusted_image.at<uchar>(y,x) = 255;
                }
                // constrast_brightness_adjusted_image.at<uchar>(y,x) = gray_image.at<uchar>(y,x) * alpha_value + beta_value;
            }
        }
       
        imshow("parameter_setting", constrast_brightness_adjusted_image);
    }
    return 0;
}

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

float Brightness;
float Contrast ;
float Saturation;
float Gain;

int B;
int C;
int S;
int G;

char winName[20]="Live";
Mat frame;
VideoCapture cap ( 1 );

void onTrackbar_changed ( int, void * )
{
        Brightness =float ( B ) /100;
        Contrast   =float ( C ) /100;
        Saturation =float ( S ) /100;
        Gain       =float ( G ) /100;

        cap.set ( CV_CAP_PROP_BRIGHTNESS,Brightness );
        cap.set ( CV_CAP_PROP_CONTRAST, Contrast );
        cap.set ( CV_CAP_PROP_SATURATION, Saturation );
        cap.set ( CV_CAP_PROP_GAIN, Gain );

}

int main ( int, char ** )
{

        if ( !cap.isOpened() ) // check if we succeeded
                return -1;

        cout<<"Press 's' to save snapshot"<<endl;
        namedWindow ( winName );

        Brightness = cap.get ( CV_CAP_PROP_BRIGHTNESS );
        Contrast   = cap.get ( CV_CAP_PROP_CONTRAST );
        Saturation = cap.get ( CV_CAP_PROP_SATURATION );
        Gain       = cap.get ( CV_CAP_PROP_GAIN );

        cout<<"===================================="<<endl<<endl;
        cout<<"Default Brightness -------> "<<Brightness<<endl;
        cout<<"Default Contrast----------> "<<Contrast<<endl;
        cout<<"Default Saturation--------> "<<Saturation<<endl;
        cout<<"Default Gain--------------> "<<Gain<<endl<<endl;
        cout<<"===================================="<<endl;

        B=int ( Brightness*100 );
        C=int ( Contrast*100 );
        S=int ( Saturation*100 );
        G=int ( Gain*100 );

        createTrackbar ( "Brightness",winName, &B, 100, onTrackbar_changed );
        createTrackbar ( "Contrast",winName, &C, 100,onTrackbar_changed );
        createTrackbar ( "Saturation",winName, &S, 100,onTrackbar_changed );
        createTrackbar ( "Gain",winName, &G, 100,onTrackbar_changed );

        int i=0;
        char name[10];
        for ( ;; ) {

                cap >> frame; // get a new frame from camera
                imshow ( "image_raw", frame );
                if ( waitKey ( 30 ) >= 0 ) break;
        }
        return 0;
}

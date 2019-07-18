#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "Image window";

class testCam{
public:
	testCam(){
		VideoCapture cap(CV_CAP_ANY); // open the default camera
	    if(!cap.isOpened())  // check if we succeeded
	        cout << "Erro loading camera" << endl;
		
		Mat edges;
	    namedWindow(OPENCV_WINDOW);

	    while(1)
	    {
	    	Mat frame;
	    	cap >> frame;
	    	cvtColor(frame, edges, COLOR_BGR2GRAY);
	    	imshow(OPENCV_WINDOW, edges);
	    	if(waitKey(30) >= 0) break;
	    }
	}

    ~testCam(){
    	destroyWindow(OPENCV_WINDOW);
    }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "StreamingCamera");
	testCam tc;
	//ros::spin();
	//return 0;
	//ROS_INFO('cam tested');
}

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <std_msgs/UInt8MultiArray.h>

void imageCallback(const std_msgs::UInt8MultiArray::ConstPtr& array)
{	   
	try
	{
		cv::Mat frame = cv::imdecode(array->data, 1);

		const int w = 1920 ;
		const int h = 1080 ;
		const float w_scale = (float)w / (float)frame.cols ;
		const float h_scale = (float)h / (float)frame.rows ;
		const float scale = cv::min(w_scale, h_scale) ;
		if( scale < 1.0 )
		{
			cv::resize(frame, frame, cv::Size(), scale, scale) ;
		}

		cv::Mat display = cv::Mat::zeros(cv::Size(w,h), CV_8UC3) ;

		cv::Rect roi_display = cv::Rect((w-frame.cols)/2,(h-frame.rows)/2,frame.cols,frame.rows) ;
		if(roi_display.x < 0 )	roi_display.x = 0 ;
		if(roi_display.y < 0 )	roi_display.y = 0 ;
		if(roi_display.x + roi_display.width > w )	roi_display.width = w - roi_display.x ;
		if(roi_display.y + roi_display.height > h )	roi_display.height = h - roi_display.y ;
		
		cv::Rect roi_from = cv::Rect(0,0,frame.cols,frame.rows) ;
		if( roi_from.width > roi_display.width )	roi_from.width = roi_display.width ;
		if( roi_from.height > roi_display.height )	roi_from.height = roi_display.height ;
		
		display(roi_display) = frame(roi_from) ;
			
		cv::imshow("view", display);
		cv::waitKey(33);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cannot decode image");
	}
}

int main(int argc, char **argv)
{
	cv::namedWindow("view", cv::WINDOW_NORMAL);
    cv::setWindowProperty("view", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	
	ros::init(argc, argv, "image_stream");

	ros::NodeHandle nhp("~");
	std::string str_node_name;
	nhp.getParam("server_name", str_node_name);
	
	ROS_INFO("Got parameter name : %s", str_node_name.c_str());

	
	//cv::namedWindow("view");
	//cv::startWindowThread();
	std::string image_node_name = str_node_name + "/image" ;
	
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(image_node_name, 5, imageCallback);

	ros::spin();
	//cv::destroyWindow("view");
}

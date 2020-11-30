#include <ros/ros.h>
#include <ros/master.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <std_msgs/UInt8MultiArray.h>

//boost 
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/regex.hpp>

#include <boost/range/iterator_range.hpp>
#include <boost/system/error_code.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/iostreams/categories.hpp>  // sink_tag
#include <iterator>  // back_inserter
#include "boost/date_time/local_time/local_time.hpp"


boost::posix_time::ptime g_check_time ;

void imageCallback(const std_msgs::UInt8MultiArray::ConstPtr& array)
{	   
	g_check_time = boost::posix_time::second_clock::local_time();
	
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

		//cv::imshow("frame", frame) ;

		cv::Mat display = cv::Mat::zeros(cv::Size(w,h), CV_8UC3) ;

		cv::Rect roi_display = cv::Rect((w-frame.cols)/2,(h-frame.rows)/2,frame.cols,frame.rows) ;
		if(roi_display.x < 0 )	roi_display.x = 0 ;
		if(roi_display.y < 0 )	roi_display.y = 0 ;
		if(roi_display.x + roi_display.width > w )	roi_display.width = w - roi_display.x ;
		if(roi_display.y + roi_display.height > h )	roi_display.height = h - roi_display.y ;
		
		cv::Rect roi_from = cv::Rect(0,0,frame.cols,frame.rows) ;
		if( roi_from.width > roi_display.width )	roi_from.width = roi_display.width ;
		if( roi_from.height > roi_display.height )	roi_from.height = roi_display.height ;
		
		//printf("roi_display = %d, %d, %d, %d\n", roi_display.x, roi_display.y, roi_display.width, roi_display.height) ;
		//printf("roi_from = %d, %d, %d, %d\n", roi_from.x, roi_from.y, roi_from.width, roi_from.height) ;

		//display(roi_display) = frame(roi_from) ;
		frame(roi_from).copyTo(display(roi_display)) ;
			
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
	ros::init(argc, argv, "image_stream");

	ros::NodeHandle nhp("~");
	std::string str_node_name;
	nhp.getParam("server_name", str_node_name);
	
	ROS_INFO("Got parameter name : %s", str_node_name.c_str());

	std::string image_node_name = str_node_name + "/image" ;

	cv::namedWindow("view", cv::WINDOW_NORMAL);
    cv::setWindowProperty("view", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	g_check_time = boost::posix_time::second_clock::local_time();
	while(1)
	{
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe(image_node_name, 5, imageCallback);

		while (ros::ok())
	    {
	        /*...TODO...*/ 

			//check time
			boost::posix_time::ptime d_time = boost::posix_time::second_clock::local_time();
			boost::posix_time::time_duration td = d_time - g_check_time;
			// result in ms
			//uint64_t ms = dur.total_milliseconds();
			// result in usec
			//uint64_t us = dur.total_microseconds();
			// result in sec
			uint64_t d_s = td.total_seconds();
			if( d_s >= 5 )	
			{
				printf("timeout(%d)\n", (int)d_s);
				sub.shutdown() ;
				break ;
			}

	        ros::spinOnce();
			boost::this_thread::sleep(boost::posix_time::millisec(33));

			//if( sub.getNumPublishers() ==  0) 
			//{
			//	printf("getNumPublishers(%d)\n", sub.getNumPublishers());
			//	sub.shutdown() ;
			//	break ;
			//}
	        //loop_rate.sleep();
	    }

		boost::this_thread::sleep(boost::posix_time::millisec(1));
		boost::thread::yield() ;
	}

	cv::destroyWindow("view");

	return 0 ;
	//cv::destroyWindow("view");
}

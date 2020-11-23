#include <ros/ros.h>
#include <ros/master.h>

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
	
	bool b_topic_ok = false ;	

	std::string str_check_topic_name = "/" + image_node_name ;
	printf("Checking Topic : %s\n", str_check_topic_name.c_str()) ;
	
	//wait
	while(b_topic_ok == false)
	{
		ros::master::V_TopicInfo topic_infos;
  		ros::master::getTopics(topic_infos);
	
		for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++) 
		{
			const ros::master::TopicInfo& info = *it;
			//std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
			
			if( str_check_topic_name == info.name )
			{
				//printf("topic ok\n") ;
				
				b_topic_ok = true ;
				break ;
			}
		}
	}

	printf("Checked Topic : %s - Start Program\n", str_check_topic_name.c_str()) ;

	cv::namedWindow("view", cv::WINDOW_NORMAL);
    cv::setWindowProperty("view", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe(image_node_name, 5, imageCallback);

	ros::spin();

	return 0 ;
	//cv::destroyWindow("view");
}

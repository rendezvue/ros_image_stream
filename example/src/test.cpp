#include "opencv2/opencv.hpp"
#include <stdlib.h>

int main(int argc, char *argv[])
{
	std::string str_server_name = "ensemble_nx_29" ;
	
	if( argc == 2)
	{
		str_server_name = argv[1] ;
	}

	std::string str_command = "rosrun ros_image_stream ros_image_stream _server_name:=" + str_server_name ;

	system(str_command.c_str());
	return 0 ;
}


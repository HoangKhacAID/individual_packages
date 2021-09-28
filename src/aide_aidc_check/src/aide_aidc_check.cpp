#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "fstream"
#include <visualization_msgs/Marker.h>

template <class T>
T String_To_Num(std::string str)
{
	std::stringstream ss(str);
	T converted_int;
	ss >> converted_int;
	return converted_int;
}

int main(int argv, char** argc)
{
	ros::init(argv, argc, "aide_aidc_check_node");
	ros::NodeHandle nh;
	ros::Rate r(100);
	std::ifstream rndf_data_points_file;
	rndf_data_points_file.open("/home/aidrivers01/21-12-2020-testing/rndf_data_points.csv");
	std::string line, word;
	std::vector<double> rndf_positions;
	bool first_time = true;
	ros::Publisher rndf_position_pub = nh.advertise<visualization_msgs::Marker>("rndf_positions", 10);
	visualization_msgs::Marker points;
	points.header.frame_id = "utm";
	points.type = visualization_msgs::Marker::POINTS;
	points.id = 0;
	geometry_msgs::Point temp_point;

	points.scale.x = 0.5;
	points.scale.y = 0.5;
	points.color.g = 1;
	points.color.a = 1;

	int count = 0;
	while(std::getline(rndf_data_points_file, line))
	{
		std::stringstream ss(line);
		while(std::getline(ss, word, ','))
		{
			if (count == 0)
			{
				temp_point.x = String_To_Num<double>(word);
				count = 1;
			}
			else
			{
				temp_point.y = String_To_Num<double>(word);
				points.points.push_back(temp_point);
				count = 0;
			}
		}
	}

	while(ros::ok())
	{	
		rndf_position_pub.publish(points);
		r.sleep();
		ros::spinOnce();
	} 
}
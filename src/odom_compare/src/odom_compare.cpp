#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class odom_compare{
public:
  odom_compare(ros::NodeHandle nh)
  {
    nh_ = nh;
    sub_1_.subscribe(nh_, "odom_1", 10);
    sub_2_.subscribe(nh_, "odom_2", 10);


    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
    sync_->registerCallback(boost::bind(&odom_compare::odomCallback, this, _1, _2));
    ROS_INFO("odom_compare object is up");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr odom_1, const nav_msgs::Odometry::ConstPtr odom_2)
  {

    double time_difference = odom_1->header.stamp.toSec() - odom_2->header.stamp.toSec();



    if(fabs(time_difference) > 0.02)
    {
      return;
    }
    ROS_INFO_STREAM("Time difference is: " << time_difference);
    count++;
    ROS_INFO_STREAM("count is: " << count);
    double eucledian_distance = sqrt( pow(odom_1->pose.pose.position.x - odom_2->pose.pose.position.x, 2) +
                                      pow(odom_1->pose.pose.position.y - odom_2->pose.pose.position.y, 2));

    accumulated_ = accumulated_ + eucledian_distance;
    ROS_INFO_STREAM("Average eucledian distance is: " << accumulated_/count);

    ROS_INFO_STREAM("The eucledian distance is: " << eucledian_distance);

    if(eucledian_distance > largest_value_)
    {
      largest_value_ = eucledian_distance;
      corresponding_time_difference_ = time_difference;
      corresponding_x_covariance = odom_2->pose.covariance.at(0);
      corresponding_y_covariance = odom_2->pose.covariance.at(7);
    }

    ROS_INFO_STREAM("largest value is: " << largest_value_);
    ROS_INFO_STREAM("The corresponding time difference is: " << corresponding_time_difference_);
    ROS_INFO_STREAM("The corresponding x-covariance is: " << corresponding_x_covariance);
    ROS_INFO_STREAM("The corresponding y-covariance is: " << corresponding_y_covariance);

    if(eucledian_distance > 0.10)
    {
      count_larger_than_10_cm_++;
    }
    if(eucledian_distance > 0.15)
    {
      count_larger_than_15_cm_++;
    }
    if(eucledian_distance > 0.2)
    {
      count_larger_than_20_cm_++;
    }
    ROS_INFO_STREAM("count_larger_than_10_cm_ is: " << count_larger_than_10_cm_);
    ROS_INFO_STREAM("count_larger_than_15_cm_ is: " << count_larger_than_15_cm_);
    ROS_INFO_STREAM("count_larger_than_20_cm_ is: " << count_larger_than_20_cm_);

  }

private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_1_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_2_;

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  int count = 0;
  double accumulated_ = 0;
  double largest_value_ = 0;
  double corresponding_time_difference_ = 0;
  int count_larger_than_10_cm_ = 0;
  int count_larger_than_15_cm_ = 0;
  int count_larger_than_20_cm_ = 0;
  double corresponding_x_covariance = 0;
  double corresponding_y_covariance = 0;
};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "odom_compare_node");
  ros::NodeHandle nh;
  ros::Rate r(100);
  ROS_INFO("node started");

  odom_compare dummy_object(nh);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

#include <px_comm/OpticalFlow.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define PI 3.14159

px_comm::OpticalFlow opt_flow_msg;
nav_msgs::Odometry opt_flow_odom_msg;
geometry_msgs::Quaternion opt_flow_quat;
tf::Quaternion q;

ros::Time current_time, last_time;
double vx, vy;
double x, y, z;
double roll, pitch, yaw;
double vx_sum, vy_sum;
double vx_offset, vy_offset, angle_offset;
double dt;

bool calibrated = true;

void
printTopicInfo(void)
{
    // print published/subscribed topics
    std::string nodeName = ros::this_node::getName();

    ros::V_string topics;
    ros::this_node::getSubscribedTopics(topics);

    std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
    for (unsigned int i = 0; i < topics.size(); ++i)
    {
        topicsStr += ("\t\t" + topics.at(i) + "\n");
    }

    topicsStr += "\tadvertised topics:\n";
    ros::this_node::getAdvertisedTopics(topics);
    for (unsigned int i = 0; i < topics.size(); ++i)
    {
        topicsStr += ("\t\t" + topics.at(i) + "\n");
    }

    ROS_INFO("%s", topicsStr.c_str());
}


void opt_flow_callback(const px_comm::OpticalFlow msg){
	// update velocities
	vx = msg.velocity_x - vx_offset;
	vy = msg.velocity_y - vy_offset;
	z = msg.ground_distance;
}

void ardrone_imu_callback(const sensor_msgs::Imu ardrone_imu){
  // update quaternion
  opt_flow_quat = ardrone_imu.orientation;
  quaternionMsgToTF(opt_flow_quat, q);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  yaw = 0.0;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "of_odom");

  ros::NodeHandle n;

  //tf::TransformBroadcaster opt_flow_odom_broadcaster;

  ros::Publisher opt_flow_odom_pub = n.advertise<nav_msgs::Odometry>("of_odom", 50);
  ros::Subscriber opt_flow_sub = n.subscribe("/px4flow/opt_flow", 50, opt_flow_callback);
  ros::Subscriber ardrone_imu_sub = n.subscribe("ardrone/imu", 50, ardrone_imu_callback);  

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // initialize variable to zero
  x = 0.0;
  y = 0.0;
  vx_offset= 0.0;
  vy_offset = 0.0;
  vx_sum = 0.0;
  vy_sum = 0.0;

  angle_offset = 

  dt = (current_time - last_time).toSec();

  printTopicInfo();

  int i = 0;
  if(!calibrated) ROS_INFO("Calibrating, please hold still.");
  else ROS_INFO("Calibration not enabled. Proceed.");

  ros::Rate r(20);

  while(n.ok()){
  	// process callbacks
  	ros::spinOnce();

  	// get current time
	current_time = ros::Time::now();

	if(!calibrated){
		vx_sum += vx;
		vy_sum += vy;
		i++;
		if(i == 100){
			vx_offset += vx_sum/200;
			vy_offset += vy_sum/200;
			ROS_INFO("Done calibrating, proceed.");
			ROS_INFO("Vx offset = %f",vx_offset);
			ROS_INFO("Vy offset = %f",vy_offset);
      calibrated = true;
		}
	}

	else{
		// calculate time since last loop
	    dt = (current_time - last_time).toSec();

	    // find change in position since last call
	    double delta_x = (vy * cos(yaw) - vx * sin(yaw)) * dt;
	    double delta_y = (vx * cos(yaw) + vy * sin(yaw)) * dt;

	    // update positions
	    x += delta_x;
	    y += delta_y;

	    // update optic flow odometry message
	    opt_flow_odom_msg.header.stamp = current_time;
	    opt_flow_odom_msg.header.frame_id = "/opt_flow_odom";
	    opt_flow_odom_msg.child_frame_id = "base_link";
	    opt_flow_odom_msg.pose.pose.position.x = x;
	    opt_flow_odom_msg.pose.pose.position.y = y;
	    opt_flow_odom_msg.pose.pose.position.z = z;
	    opt_flow_odom_msg.pose.pose.orientation = opt_flow_quat;
	    opt_flow_odom_msg.twist.twist.linear.x = vx;
	    opt_flow_odom_msg.twist.twist.linear.y = vy;

	    // update transform message
	//     opt_flow_odom_tf.header.stamp = current_time;
	//     opt_flow_odom_tf.header.frame_id = "/opt_flow_odom";
	//     opt_flow_odom_tf.child_frame_id = "base_link";
	//     opt_flow_odom_tf.transform.translation.x = x;
	//     opt_flow_odom_tf.transform.translation.y = y;
	//     opt_flow_odom_tf.transform.translation.z = 0.0;
	//     opt_flow_odom_tf.transform.rotation = opt_flow_quat;
	}

    // publish messages
    //opt_flow_odom_broadcaster.sendTransform(opt_flow_odom_tf);
    opt_flow_odom_pub.publish(opt_flow_odom_msg);

    last_time = current_time;

    r.sleep();
  }

  return 0;
}

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
std::string output_topic;// = "/depth_pontcloud_converted";
std::string input_topic;// = "/depth_camera/depth/points";
ros::Publisher pub;
Eigen::Matrix4f transform;


void callback(PointCloud pcl_in){
	PointCloud pcl_out;
	pcl::transformPointCloud(pcl_in, pcl_out, transform);
	pub.publish(pcl_out);
}

void init(float x,float y,float z,float w){
	const tf::Quaternion q(x,y,z,w);
	tf::Transform bt;
	bt.setRotation(q);
	pcl_ros::transformAsMatrix(bt, transform);
}

int main(int argc, char** argv){
	ros::init (argc, argv, "transform_pcl_node");
	ros::NodeHandle nh;
	float x,y,z,w;
	if (nh.getParam("/transform_node/topic_in", input_topic)
		&& nh.getParam("/transform_node/topic_out", output_topic)
		&& nh.getParam("/transform_node/rotation_x", x)
		&& nh.getParam("/transform_node/rotation_y", y)
		&& nh.getParam("/transform_node/rotation_z", z)
		&& nh.getParam("/transform_node/rotation_w", w)
		){	
		init(x,y,z,w);
	}
	else
		ROS_FATAL("%s", "Wrong parameters");
	
	pub = nh.advertise<PointCloud> (output_topic, 3);
	ros::Subscriber sub = nh.subscribe(input_topic, 3, callback);
	ros::spin();
	return 0;
}

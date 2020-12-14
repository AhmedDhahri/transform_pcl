# transform_pcl
Apply 3D transformation to "sensor_msgs/PointCloud2" ROS topics.

# Paramaters: 
*the config file is cfg/config.yaml*

- topic_in: input topic to transform.
- topic_out: output transformed topic.
- rotation_x: component x of the quaternion rotation.
- rotation_y: component y of the quaternion rotation.
- rotation_z: component z of the quaternion rotation.
- rotation_w: component w of the quaternion rotation.
- translation_x: component x of the translation.
- translation_y: component y of the translation.
- translation_z: component z of the translation.

# Launch file
*launch/transform.launch*


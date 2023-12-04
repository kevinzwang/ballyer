# ballyer

transform from AR tag to base: 
```rosrun tf2_ros static_transform_publisher 0  0.127 -.13 -1.57 -1.57 0 ar_marker_15 base```

get rectified image:
```ROS_NAMESPACE=usb_cam rosrun image_proc image_proc```
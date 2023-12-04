# ballyer

start usb cam:
```roslaunch vision run_cam.launch```

start AR tracking:
```roslaunch vision ar_track.launch```

transform from AR tag to base: 
```rosrun tf2_ros static_transform_publisher 0  0.127 -.13 -1.57 -1.57 0 ar_marker_15 base```

get rectified image:
```ROS_NAMESPACE=usb_cam rosrun image_proc image_proc```

detect object:
```rosrun vision object_detection2.py```
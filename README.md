The traditional method of logging RGBD data involves saving each frame separately while the camera is active. RGB and depth images are saved individually, with depth information often stored as an alpha channel in the RGBA pixel format. This alpha channel, originally used for transparency, can alternatively serve as a fourth channel to store both RGB and depth data together. This was done in rgbd_save.py amd rgbd_print.py in this repo. 

The LiDAR data were saved and published in scan_save.py and scan_print.py. These 2D LiDAR scanning data saves in .csv format with all the published categories, and the points are transformed into x and y coordinates respectively. 

The encoder.py file is designed to subscribe to encoder data received via a Serial connection from devices like an Arduino, ESP32, or Teensy. It extracts this data and publishes it to ROS2 topics. On the other hand, the lidar_encoder_fusion.py file combines the encoder data, published at 100 Hz, with LiDAR scanning data, which is published at 15 Hz. This fusion is primarily based on matching the closest timestamps between the encoder and LiDAR data. Following this, the fusion_filtered.py script applies a particle filter to eliminate noise and irregular scans detected in the earlier fusion process.

The files can be built with a local directory: 

```
mkdir tf_ws
cd tf_ws
mkdir src
cd src 
git clone https://github.com/jasonqiuwo/data_processing.git 
colcon build --symlink-install 
```

The file format used for logging camera data is .h5 files, which organizes the saved data in a hierarchical structure. This method allows for easy loading and integration into ROS2 nodes, where the data can be subscribed to and visualized in Rviz2. Below is an example of how frames can be saved in this format, and this could be read by using read_h5.py:

```
depth: [0,  0,  0,  0,  0,  0
        ...
        2628, 2639, 2639, 2650, 2650, 2662]
dtype=uint16

rgb: [[ 97 124  70] [ 97 124  70] [ 94 123  69]
      ...
      [ 51  79  41] [ 52  80  42] [ 51  79  41]]
```

Here is an example of saving a few constant frames of camera live data and then publish them to a ROS2 topic. Then this topic can be subscribed from the RViz side. 

![rgbdrecon](https://github.com/user-attachments/assets/9c77b762-085a-4c21-90dd-fbb8f23bad20)


The saving and publishing of the LiDAR data is in .csv files. The data saved can then be published to the pointcloud which can be viewed in Rviz2. A brief example is shown below as it is integrated with a ROS2 node. 

![pointcloud_scan_data](https://github.com/user-attachments/assets/ebbbccc2-261f-440d-a573-4382562f5d3f)


These Python scripts can be ran as follows. Also make sure that the LiDAR or the RGBD camera is connected and the respective nodes is running. 
```
ros2 run my_transforms scan_save.py
```


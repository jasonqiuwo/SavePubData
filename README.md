The traditional method of logging RGBD data involves saving each frame separately while the camera is active. RGB and depth images are saved individually, with depth information often stored as an alpha channel in the RGBA pixel format. This alpha channel, originally used for transparency, can alternatively serve as a fourth channel to store both RGB and depth data together.

The file format used for logging is .h5 files, which organize data in a hierarchical structure. This method allows for easy loading and integration into ROS2 nodes, where the data can be subscribed to and visualized in Rviz2. Below is an example of how frames can be saved in this format:

depth: [   0    0    0    0    0    0  ...  2628, 2639, 2639, 2650, 2650, 2662] dtype=uint16

rgb: [[ 97 124  70] [ 97 124  70] [ 94 123  69]  ...  [ 51  79  41] [ 52  80  42] [ 51  79  41]]

![rgbdrecon](https://github.com/user-attachments/assets/9c77b762-085a-4c21-90dd-fbb8f23bad20)

# ME495 Sensing, Navigation and Machine Learning For Robotics
* Megan Black
* Winter 2024
# Package List
This repository consists of several ROS packages
- nuturtle_description - Displays multiple turtlebot3 models in rviz
- nusim - Provides simulation environment
- nuturtle_control - Provides control of the turtlebot3
- nuslam - Provides a SLAM implementation for the turtlebot3

# Videos

### Real robot with unknown data association:

[SLAM_real_robot.webm](https://github.com/ME495-Navigation/slam-project-blackm159/assets/116540591/ead5332e-18cd-4497-851a-34eecb968686)

[SLAM_real.webm](https://github.com/ME495-Navigation/slam-project-blackm159/assets/116540591/45c100a0-c512-4d88-bc22-9bd54964be4c)

The final pose error between the actual robot (red) and odometry (blue) is [0.235, -0.013] m and -0.104 rad.  The final pose error between the actual robot (red) and the estimated robot (green) is [0.113, 0.010] m and -0.034 rad.


### Robot with unknown data association in RViz:

[SLAM_unknown_data_assoc.webm](https://github.com/ME495-Navigation/slam-project-blackm159/assets/116540591/3714522b-6e7b-4b87-8d56-bd5664c8c15f)

The final pose error between the actual robot (red) and odometry (blue) is [0.121, -0.114] m and 0.392 rad.  The final pose error between the actual robot (red) and the estimated robot (green) is [0.027, 0.029] m and 0.04 rad.


### Robot with known data association in RViz:
![Robot in rviz](nuslam/images/slam_in_rviz.png)



### Robot moving in a circle:

[HW2 video.webm](https://github.com/ME495-Navigation/slam-project-blackm159/assets/116540591/ec95c278-97a4-44f5-a1f7-31fcfc5b0a10)

[real_robot.webm](https://github.com/ME495-Navigation/slam-project-blackm159/assets/116540591/963bf695-9418-47d8-ad15-d5317a553ab3)

Although the real robot returns to the starting position at the end of the video, the odometry robot in rviz ends up 0.747m away from the starting position.
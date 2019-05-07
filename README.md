# arhmd_study_dre

This respository was created for the AIR lab study conducted at Tufts University.
In this project,  we modify a mobile robot with augmented reality (AR) signals to reveal navigation plans to an individual with an AR device.  In the photo, a person wearing a Microsoft HoloLens is able to see the Turtlebot’s costmap (dotted grid), path trajectory (yellow dotted line), and laser scan data (red dots). We are conducting a human study to see how participants perceive and utilize this data when encountering a robot seen for the first time.

In this repository, I created scripts to control the Turtlebots position within the map of a hallway located in 200 Boston. The map is located in the Tufts Map folder. The Turtlebot will localize first, then move to the position of point A. The robot will remain at point A until a user provides the participant ID and the selected Mode for the participant. This information will be used to name the rosbag file for the data of interest for the study. 

The robot will then proceed to point B which is located at the the beginning of the Hallway, and then traverse to point C which is located at the end of the Hallway. Points B and C differ in the y direction so that the planner forces the robot to move in a straight line within the middle of the corridor. 

To run the study, lunch the scripts in the following order:

1) launch tbot2_dre.launch 
2) Then rosrun blink.py. 
3) Then rosrun 200_Boston_ABCD.py 
4) Then rosrun data_recorder.py

 

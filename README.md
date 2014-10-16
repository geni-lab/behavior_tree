behavior_tree
=============
Behavior tree for Zoidstein and Zeno

Prerequisites
=============
- Download and install Owyl: http://code.google.com/p/owyl/downloads/list

Usage
=============
Clone into your catkin workspace and run:

rosrun behavior_tree behavior_tree.py

It listens on a list of topics:
- itf_listen, for audio input
- zenodial_talk, for output from zenodial
- speech_active, for checking if the robot is currently speaking
- facedetect, for faces detected
- emo_pub, for emotion input
- affect_pub, for emotion input
- /image_detect, for object recognition

It publishes on:
- itf_talk, for the robot to speak
- zenodial_listen, send the message to zenodial
- robot_movement, for robot movement
- itf_talk_stop, to stop the speech

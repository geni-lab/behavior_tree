#!/usr/bin/python

##
# This is an extract from "behavior_node_master.py" for testing purpose
# The complete behavior tree logic is still in "behavior_node_master.py"
##

import roslib
import rospy
import math
import os
import time
import std_msgs
import random
import owyl
import sys

from owyl import blackboard
from simple_face_tracker.msg import targets
from std_msgs.msg import String, Bool
from sensor_msgs.msg import RegionOfInterest

# Jamie's API
from hri_api.entities import Person, World, Saliency
from zoidstein_hri.zoidstein import Zoidstein, Gesture
from zeno_hri.zeno import Zeno, ZenoGesture
from hri_common import Expression
from hri_api.query import Query

class Tree:
    def __init__(self, tree_name):
        self.blackboard = blackboard.Blackboard()

        rospy.Subscriber("itf_listen", String, self.audioInputCallback)
        rospy.Subscriber("zenodial_talk", String, self.zenoDialCallback)
        rospy.Subscriber("speech_active", Bool, self.isSpeakingCallback)
        rospy.Subscriber("facedetect", targets, self.faceDetectCallback)
        # rospy.Subscriber("/nmpt_saliency_point", targets, self.saliencyCallback)
        rospy.Subscriber("emo_pub", String, self.emoCallback)
        rospy.Subscriber("affect_pub", String, self.affectCallback)
        rospy.Subscriber("/image_detect", String, self.objectRecognitionCallback)
        self.itf_talk_pub = rospy.Publisher("itf_talk", String, queue_size=1)
        self.zenodial_listen_pub = rospy.Publisher("zenodial_listen", String, queue_size=1)
        self.robot_movement_pub = rospy.Publisher("robot_movement", String, queue_size=1)
        self.itf_talk_stop_pub = rospy.Publisher("itf_talk_stop", String, queue_size=1)
        self.itf_robot_name_pub = rospy.Publisher("itf_robot_name", String, queue_size=1)
        self.cmd_blendermode_pub = rospy.Publisher("cmd_blendermode", String, queue_size=1)
        self.roi_pub = rospy.Publisher("roi", RegionOfInterest, queue_size=1)

        self.blackboard["commandKeywords"] = {
                'Walking Forward': ['walk for', 'forward', 'go forward', 'move forward', 'go ahead', 'move ahead', 'go straight', 'move straight', 'go forwards', 'move forwards', 'walk forward', 'walk forwards'],
                'Walking Backward': ['backward', 'go back', 'move back', 'walk back', 'go backward', 'move backward', 'walk backward', 'go backwards', 'move backwards', 'walk backwards'],
                'Turning Left': ['turn left', 'turn lefts', 'turns left'],
                'Turning Right': ['turn right', 'turn rights', 'turns right'],
                'Stop Speaking': ['stop speaking', 'shut up'],
                'Smile': ['smile'],
                'Frown': ['frown'],
                # 'Frown Mouth': ['frown mouth'],
                'Surprise': ['surprise'],
                'Take This': ['take this', 'take it', 'take that'],
                'Give Back': ['give it back', 'give that back', 'give back', 'give it'],
                'Wave': ['wave']}
        self.blackboard["playemotion"] = {
                "Start Emotion Detection": ["start emotion recognition", "start emotion detection", "play emotion detection", "start playing emotion detection", "play emotion game", "play emotion detection game", "start emotion identification"]}
        self.blackboard["stopplayemotion"] = {
                "Stop Emotion Detection": ["stop emotion recognition", "stop emotion detection", "stop emotion game", "stop playing emotion detection", "stop playing emotion game", "stop emotion identification"]}
        self.blackboard["playObjRecognition"] = {
                "Start Object Recognition": ["start object recognition", "play object recognition", "start object identification", "play object identification", "start object detection"]}
        self.blackboard["stopObjRecognition"] = {
                "Stop Object Recognition": ["stop object recognition", "stop object identification", "stop object detection"]}

        ### Inputs
        self.blackboard["frontFaceTarget"] = {}
        self.blackboard["faceTarget"] = {}                        # seq no: [Person obj, x, y, velocity, age, disappear_age]
        self.blackboard["frontSaliencyTarget"] = {}
        self.blackboard["saliencyTarget"] = {}                    # seq no: [Person obj, x, y, velocity, age, disappear_age]
        self.blackboard["frontAudioInput"] = ""
        self.blackboard["audioInput"] = ""                        # string output of speech-to-text algorithm, raw form
        self.blackboard["audioInputAge"] = 0                      # time since the last significant parse of the audio input
        self.blackboard["audioInputVol"] = 0                      # average volume or magnitude of the last audio input
        self.blackboard["frontRosInput"] = ""
        self.blackboard["rosInput"] = ""                          # string representation of miscellaneous commands from other ros components, usually blender
        self.blackboard["rosInputAge"] = 0                        # time since the last ros command
        self.blackboard["frontEmoInput"] = ""
        self.blackboard["emoInput"] = ""
        self.blackboard["frontAffectInput"] = ""
        self.blackboard["affectInput"] = ""
        self.blackboard["emotionInputAge"] = 0                    # time since the last significant chance in emotional state
        self.blackboard["speechOutputAge"] = 0                    # time since the last speech output from the robot
        self.blackboard["randomInput"] = 0                        # a random percentile for random behaviors
        self.blackboard["randomInput2.5"] = self.blackboard["randomInput"] * 2.5

        ### Globals
        self.blackboard["blinkChance"] = 0.011                     # @ 60 fps a 1.1% chance to start a blink each frame should give us a nice frequency
        self.blackboard["blinkChance1.5"] = self.blackboard["blinkChance"] * 1.5
        self.blackboard["blinkChance1.2"] = self.blackboard["blinkChance"] * 1.2
        self.blackboard["eyeFreedom"] = 0.5
        self.blackboard["neckFreedom"] = 0.5
        self.blackboard["StopSpeech"] = "Stop Speech"
        self.blackboard["WalkForward"] = "Walking Forward"
        self.blackboard["WalkBackward"] = "Walking Backward"
        self.blackboard["TurnLeft"] = "Turning Left"
        self.blackboard["TurnRight"] = "Turning Right"
        self.blackboard["Wave"] = "Wave"
        self.blackboard["Smile"] = "Smile"
        self.blackboard["Frown"] = "Frown"
        # self.blackboard["FrownMouth"] = "Frown Mouth"
        self.blackboard["Surprise"] = "Surprise"
        self.blackboard["TakeThis"] = "Take This"
        self.blackboard["GiveBack"] = "Give Back"
        self.blackboard["StopSpeaking"] = "Stop Speaking"
        self.blackboard["eyeFree0.25"] = self.blackboard["randomInput"] * 0.25 * self.blackboard["eyeFreedom"]
        self.blackboard["eyeFree0.75"] = self.blackboard["randomInput"] * 0.75 * self.blackboard["eyeFreedom"]
        self.blackboard["neckFree0.1"] = self.blackboard["randomInput"] * 0.1 * self.blackboard["neckFreedom"]
        self.blackboard["neckFree0.3"] = self.blackboard["randomInput"] * 0.3 * self.blackboard["neckFreedom"]
        self.blackboard["isDetectingEmotion"] = False
        self.blackboard["emotionDetectionStart"] = False
        self.blackboard["emotionDetectionEnd"] = False
        self.blackboard["frontObjectRecognized"] = ""
        self.blackboard["objectRecognized"] = ""
        self.blackboard["isRecognizingObject"] = ""
        self.blackboard["objectRecognitionStart"] = ""
        self.blackboard["objectRecognitionEnd"] = ""
        self.blackboard["boolean_true"] = True
        self.blackboard["boolean_false"] = False
        self.blackboard["null"] = ""
        self.blackboard["lastemotion"] = ""

        ### Locals
        self.blackboard["actionName"] = ""
        self.blackboard["targetPos"] = ""
        self.blackboard["firstGreeting"] = False
        self.blackboard["frontSpeechActive"] = False
        self.blackboard["speechActive"] = False
        self.blackboard["robotName"] = ""

        # Make the tree
        if tree_name == "BasicZenoTree":
            self.blackboard["robot"] = Zeno()
            self.blackboard["robotName"] = "Zeno"
            self.itf_robot_name_pub.publish(self.blackboard["robotName"])
            self.tree = self.makeBasicTree()
        elif tree_name == "BasicZoidSteinTree":
            self.blackboard["robot"] = Zoidstein()
            self.blackboard["robotName"] = "Zoidstein"
            self.itf_robot_name_pub.publish(self.blackboard["robotName"])
            self.tree = self.makeBasicTree()
        while True:
            self.tree.next()

    def makeBasicTree(self):
        robotTree = \
            owyl.parallel(
                ######################################## BodySubtree ########################################
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.sequence(
                            self.updateFrontVariables(),
                            self.determineCurrentTarget(),
                            self.removeFace(),
                            owyl.selector(
                                # Gaze at face targets
                                owyl.sequence(
                                    self.isFaceTarget(),
                                    self.isNoSalientTarget(),
                                    self.isNoAudioInput(),
                                    self.isNoRosInput(),
                                    self.isNoEmotionInput(),
                                    self.faceGaze()
                                ),

                                # Gaze at salient targets
                                # owyl.sequence(
                                #     self.isSalientTarget(),
                                #     # self.isNoFaceTarget(),
                                #     self.isNoAudioInput(),
                                #     self.isNoRosInput(),
                                #     self.isNoEmotionInput(),
                                #     self.faceGaze()
                                # ),

                                # Handle commands
                                owyl.sequence(
                                    owyl.selector(
                                        self.isAudioInput(),
                                        self.isRosInput()
                                    ),
                                    owyl.selector(
                                        self.isCommand(key="audioInput"),
                                        self.isCommand(key="rosInput")
                                    ),
                                    owyl.selector(
                                        owyl.sequence(
                                            owyl.selector(
                                                self.isCommandPhrase(commandName="StopSpeech", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="WalkForward", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="WalkBackward", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="TurnLeft", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="TurnRight", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="StopSpeaking", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="Smile", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="Frown", actionPhrase="actionName"),
                                                # self.isCommandPhrase(commandName="FrownMouth", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="Surprise", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="TakeThis", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="GiveBack", actionPhrase="actionName"),
                                                self.isCommandPhrase(commandName="Wave", actionPhrase="actionName"),
                                            ),
                                            self.isNotSpeaking(),
                                            # self.sayStartAction(key="actionName"),
                                            self.showAction(key="actionName")
                                        ),
                                        self.printStatus(msg="I'm sorry, Dave, I'm afraid I can't do that...")
                                    )
                                ),

                                # Play emotion detection game
                                owyl.sequence(
                                    owyl.selector(  #TODO: change to sequence
                                        self.isAudioInput(),
                                        self.isEmotionInput(),
                                    ),
                                    self.isNotStopEmotionDetection(key="audioInput"),
                                    self.isEmotionDetection(key="audioInput"),
                                    self.isNotSpeaking(),
                                    self.startEmotionDetection(),
                                ),

                                # Play object recognition game
                                owyl.sequence(
                                    owyl.selector(
                                        self.isAudioInput(),
                                        self.isObjInput(),
                                    ),
                                    self.isNotStopObjRecognition(key="audioInput"),
                                    self.isObjRecognition(key="audioInput"),
                                    self.isNotSpeaking(),
                                    self.startObjRecognition()
                                ),

                                # Send to the dialogue system
                                owyl.sequence(
                                    self.isAudioInput(),
                                    self.isNotSpeaking(),
                                    self.toZenoDial(key="audioInput")
                                )
                            )
                        )
                    ),
                    limit_period=0.001
                ),

                ######################################## General tree ########################################
                owyl.limit(
                    owyl.repeatAlways(
                        owyl.sequence(
                            self.test(),
                            self.updateVariables()
                        )
                    ),
                    limit_period=0.001
                ),
                policy=owyl.PARALLEL_SUCCESS.REQUIRE_ALL
            )
        return owyl.visit(robotTree, blackboard=self.blackboard)

    def zenoDialCallback(self, data):
        # self.itf_talk_pub.publish(data.data)  # For now
        self.blackboard["robot"].say(data.data)
        self.blackboard["speechOutputAge"] = 0
        print "(From ZenoDial) " + data.data

    def audioInputCallback(self, data):
        self.blackboard["audioInputAge"] = 0
        if data.data == "BADINPUT":
            print "BADINPUT"
            return
        self.blackboard["audioInput"] = data.data
        print "(From itf_listen) " + data.data

    def isSpeakingCallback(self, data):
        if data.data:
            self.blackboard["speechActive"] = data.data
        # Just to avoid changing it to False when the robot is saying one of the sentences of a long utterance
        elif self.blackboard["speechOutputAge"] > 4:
            self.blackboard["speechActive"] = data.data

    def faceDetectCallback(self, data):
        person_id = 0
        threshold = 100

        # Indexes of person_detail:
        index_person = 0
        index_x = 1
        index_y = 2
        index_vel = 3
        index_age = 4
        index_dage = 5

        # Loop through each of the people from the input
        for person in range(len(data.positions)):
            x = data.positions[person].x
            y = data.positions[person].y

            # Create a new person if there is none in the system
            if len(self.blackboard["faceTarget"]) == person_id:
                self.blackboard["faceTarget"][person_id] = [Person(person_id), x, y, 0, 0, 0]
                print "New Person(" + str(person_id) + ")"
                person_id += 1
                continue

            velocity = math.sqrt(math.pow(x - self.blackboard["faceTarget"][person_id][index_x], 2) + math.pow(y - self.blackboard["faceTarget"][person_id][index_y], 2))

            # Same Person, update his coordinates
            if velocity < threshold:
                self.blackboard["faceTarget"][person_id] = [self.blackboard["faceTarget"][person_id][index_person], x, y, velocity, self.blackboard["faceTarget"][person_id][index_age], 0]

            person_id += 1

    # def saliencyCallback(self, data):
    #     self.blackboard["saliencyTarget"][0] = [Person(0), data.positions[0].x, data.positions[0].y, 0, 0]
    #     # self.blackboard["robot"].gaze_and_wait(Person(0).head, speed=0.5)
    #     del self.blackboard["saliencyTarget"][0]

    def emoCallback(self, data):
        self.blackboard["emoInput"] = data.data
        print "emo = " + self.blackboard["emoInput"]

    def affectCallback(self, data):
        self.blackboard["affectInput"] = data.data
        print "affect = " + self.blackboard["affectInput"]

    def objectRecognitionCallback(self, data):
        self.blackboard["objectRecognized"] = data.data
        # print "Object recognized = " + self.blackboard["objectRecognized"]

    def getTheYoungestPerson(self):
        youngest_age = -1
        youngest_person = ""
        for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
            if youngest_age < 0 or person_detail[4] < youngest_age:
                youngest_age = person_detail[4]  # person_detail[4] = age
                # youngest_person = person_detail[0]  # person_detail[0] = person object
                youngest_person = [person_detail[1], person_detail[2]]  # 20140925, person_detail[1] = x, person_detail[2] = y
        return youngest_person

    @owyl.taskmethod
    def determineCurrentTarget(self, **kwargs):
        self.blackboard["targetPos"] = self.getTheYoungestPerson()
        # print str(len(self.blackboard["faceTarget"])) + " - In current target function: " + str(youngest_age)
        yield True

    @owyl.taskmethod
    def removeFace(self, **kwargs):
        # print "Looking for faces to remove"
        disappear_threshold = 250
        people_to_del = []
        for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
            if person_detail[5] > disappear_threshold:  # person_detail[5] = disappear_age
                people_to_del.append(key)
        for key in people_to_del:
            del self.blackboard["faceTarget"][key]
            print "Person(" + str(key) + ") is gone!!! (" + str(len(self.blackboard["faceTarget"])) + " left)"
        yield True

    @owyl.taskmethod
    def test(self, **kwargs):
        # print "The tree is running..." + time.strftime("%Y%m%d%H%M%S")
        # self.blackboard["robot"].say("Testing")
        # self.blackboard["robot"].expression(Expression.Smile, 1.0)
        # self.blackboard["robot"].expression(Expression.OpenMouth, 0.0)
        # self.blackboard["robot"].expression(Expression.Frown, 0.5)
        # self.blackboard["robot"].expression(Expression.FrownMouth, 0.0)
        # sys.exit()
        # elif actionName == "Frown":
        #     self.blackboard["robot"].expression(Expression.Frown, 1.0)
        #     self.blackboard["robot"].expression(Expression.FrownMouth, 1.0)
        #     self.blackboard["robot"].expression(Expression.Smile, 0.0)
        # # elif actionName == "Frown Mouth":
        # #     self.blackboard["robot"].expression(Expression.FrownMouth, 1.0)
        # elif actionName == "Surprise":
        #     self.blackboard["robot"].expression(Expression.Frown, 0.0)
        #     self.blackboard["robot"].expression(Expression.Smile, 0.5)
        #     self.blackboard["robot"].expression(Expression.OpenMouth, 1.0)
        yield True

    @owyl.taskmethod
    def printStatus(self, **kwargs):
        print kwargs["msg"]
        yield True

    @owyl.taskmethod
    def updateFrontVariables(self, **kwargs):
        self.blackboard["frontFaceTarget"] = self.blackboard["faceTarget"]
        self.blackboard["frontSaliencyTarget"] = self.blackboard["saliencyTarget"]
        self.blackboard["frontAudioInput"] = self.blackboard["audioInput"]
        self.blackboard["frontRosInput"] = self.blackboard["rosInput"]
        self.blackboard["frontEmoInput"] = self.blackboard["emoInput"]
        self.blackboard["frontAffectInput"] = self.blackboard["affectInput"]
        self.blackboard["frontSpeechActive"] = self.blackboard["speech_active"]
        self.blackboard["frontObjectRecognized"] = self.blackboard["objectRecognized"]
        yield True

    @owyl.taskmethod
    def updateVariables(self, **kwargs):
        # Indexes of person_detail:
        index_person = 0
        index_x = 1
        index_y = 2
        index_vel = 3
        index_age = 4
        index_dage = 5

        self.blackboard["randomInput"] = random.random()
        for (key, person_detail) in self.blackboard["faceTarget"].iteritems():
            self.blackboard["faceTarget"][key] = [person_detail[index_person], person_detail[index_x], person_detail[index_y], person_detail[index_vel], person_detail[index_age]+1, person_detail[index_dage]+1]
        for (key, person_detail) in self.blackboard["saliencyTarget"].iteritems():
            self.blackboard["saliencyTarget"][key] = [person_detail[index_person], person_detail[index_x], person_detail[index_y], person_detail[index_vel], person_detail[index_age]+1, person_detail[index_dage]+1]
        self.blackboard["audioInputAge"] += 1
        self.blackboard["rosInputAge"] += 1
        self.blackboard["emotionInputAge"] += 1
        self.blackboard["speechOutputAge"] += 1
        self.blackboard["eyeFree0.25"] = self.blackboard["randomInput"] * 0.25 * self.blackboard["eyeFreedom"]
        self.blackboard["eyeFree0.75"] = self.blackboard["randomInput"] * 0.75 * self.blackboard["eyeFreedom"]
        self.blackboard["neckFree0.1"] = self.blackboard["randomInput"] * 0.1 * self.blackboard["neckFreedom"]
        self.blackboard["neckFree0.3"] = self.blackboard["randomInput"] * 0.3 * self.blackboard["neckFreedom"]
        self.blackboard["blinkChance1.5"] = self.blackboard["blinkChance"] * 1.5
        self.blackboard["blinkChance1.2"] = self.blackboard["blinkChance"] * 1.2
        self.blackboard["randomInput2.5"] = self.blackboard["randomInput"] * 2.5

        # self.determineCurrentTarget()
        # self.removeFace()
        yield True

    @owyl.taskmethod
    def isLess(self, **kwargs):
        if isinstance(kwargs["num1"], (int, long)):
            num1 = kwargs["num1"]
        else:
            num1 = self.blackboard[kwargs["num1"]]
        if isinstance(kwargs["num2"], (int, long)):
            num2 = kwargs["num2"]
        else:
            num2 = self.blackboard[kwargs["num2"]]
        if num1 < num2:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isGreater(self, **kwargs):
        if isinstance(kwargs["num1"], (int, long)):
            num1 = kwargs["num1"]
        else:
            num1 = self.blackboard[kwargs["num1"]]
        if isinstance(kwargs["num2"], (int, long)):
            num2 = kwargs["num2"]
        else:
            num2 = self.blackboard[kwargs["num2"]]
        if num1 > num2:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isSpeaking(self):
        yield self.blackboard["speechActive"]

    @owyl.taskmethod
    def say(self, **kwargs):
        # self.itf_talk_pub.publish(kwargs["utterance"])  # For now
        self.blackboard["robot"].say(kwargs["utterance"])
        self.blackboard["speechOutputAge"] = 0
        yield True

    @owyl.taskmethod
    def sayStartAction(self, **kwargs):
        print "Okay, " + self.blackboard[kwargs["key"]] + "..."
        # self.itf_talk_pub.publish("Okay, " + self.blackboard[kwargs["key"]] + "...")  # For now
        self.blackboard["robot"].say("Okay, " + self.blackboard[kwargs["key"]] + "...")
        yield True

    @owyl.taskmethod
    def showAction(self, **kwargs):
        actionName = self.blackboard[kwargs["key"]]

        if actionName == "Smile":
            self.blackboard["robot"].expression(Expression.Smile, 1.0)
            self.blackboard["robot"].expression(Expression.OpenMouth, 0.0)
            self.blackboard["robot"].expression(Expression.Frown, 0.5)
        elif actionName == "Frown":
            self.blackboard["robot"].expression(Expression.Frown, 1.0)
            self.blackboard["robot"].expression(Expression.FrownMouth, 1.0)
            self.blackboard["robot"].expression(Expression.Smile, 0.0)
        # elif actionName == "Frown Mouth":
        #     self.blackboard["robot"].expression(Expression.FrownMouth, 1.0)
        elif actionName == "Surprise":
            self.blackboard["robot"].expression(Expression.Frown, 0.0)
            self.blackboard["robot"].expression(Expression.Smile, 0.5)
            self.blackboard["robot"].expression(Expression.OpenMouth, 1.0)
        elif actionName == "Wave":
            self.blackboard["robot"].gesture(Gesture.WaveHands)
        elif actionName == "Walking Forward":
            self.blackboard["robot"].gesture(Gesture.WalkForward5)
        elif actionName == "Walking Backward":
            self.blackboard["robot"].gesture(Gesture.WalkReverse2)
        elif actionName == "Turning Left":
            self.blackboard["robot"].gesture(Gesture.WalkLeftTurnInPlace)
        elif actionName == "Turning Right":
            self.blackboard["robot"].gesture(Gesture.WalkRightTurnInPlace)
        elif actionName == "Take This":
            self.blackboard["robot"].gesture(Gesture.RightArmGrab)
        elif actionName == "Give Back":
            self.blackboard["robot"].gesture(Gesture.RightArmGive)
        elif actionName == "Stop Speaking":
            self.itf_talk_stop_pub.publish("Stop...")

        self.blackboard["audioInput"] = ""
        self.blackboard["actionName"] = ""
        yield True

    @owyl.taskmethod
    def faceTrack(self, **kwargs):
        # Should be identical to faceGaze() if both of the functions use the youngest person as the target
        personToTrack = self.getTheYoungestPerson()
        # self.blackboard["robot"].gaze_and_wait(personToTrack.head, speed=0.5)
        yield True

    @owyl.taskmethod
    def faceGaze(self, **kwargs):
        # self.blackboard["robot"].gaze_and_wait(self.blackboard["targetPos"].head, speed=0.5)

        #################### 20140925 Changes ####################
        # screen_height = 480
        # screen_width = 640
        # region_height = 50
        # region_width = 50
        # x = self.blackboard["targetPos"][0]
        # y = self.blackboard["targetPos"][1]
        # x_offset = x - region_width/2
        # y_offset = y + region_height/2
        # self.cmd_blendermode_pub.publish("TrackDev")
        # self.roi_pub.publish(x_offset, y_offset, screen_height, screen_width, False)
        # self.cmd_blendermode_pub.publish("Dummy")
        # print "ROI published: (" + str(x_offset) + ", " + str(y_offset) + ", " + str(screen_height) + ", " + str(screen_width) + ")"
        #################### 20140925 Changes ####################

        self.blackboard["targetPos"] = ""
        self.blackboard["saliencyTarget"] = {}
        yield True

    @owyl.taskmethod
    def toZenoDial(self, **kwargs):
        utterance = self.blackboard[kwargs["key"]]
        print "Sending \"" + utterance + "\" to ZenoDial"
        self.zenodial_listen_pub.publish(utterance)
        self.blackboard["audioInput"] = ""
        yield True

    @owyl.taskmethod
    def isCommand(self, **kwargs):
        audiorosInput = self.blackboard[kwargs["key"]]
        found = False
        for (key, keywords) in self.blackboard["commandKeywords"].iteritems():
            for word in keywords:
                if audiorosInput.find(word) > -1:
                    found = True
                    self.blackboard["actionName"] = key
                    print audiorosInput + " is a command!!!"
                    yield True
        if not found:
            yield False

    @owyl.taskmethod
    def isCommandPhrase(self, **kwargs):
        if self.blackboard[kwargs["commandName"]] == self.blackboard[kwargs["actionPhrase"]]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNotStopEmotionDetection(self, **kwargs):
        if not self.blackboard["isDetectingEmotion"]:
            yield True
            return

        audioInput = self.blackboard[kwargs["key"]]
        for (key, keywords) in self.blackboard["stopplayemotion"].iteritems():
            for word in keywords:
                if audioInput.find(word) > -1:
                    self.blackboard["emotionDetectionEnd"] = True
        yield True

    @owyl.taskmethod
    def isEmotionDetection(self, **kwargs):
        if self.blackboard["isDetectingEmotion"]:
            yield True
            return

        audioInput = self.blackboard[kwargs["key"]]
        found = False
        for (key, keywords) in self.blackboard["playemotion"].iteritems():
            for word in keywords:
                if audioInput.find(word) > -1:
                    found = True
                    self.blackboard["isDetectingEmotion"] = True
                    self.blackboard["emotionDetectionStart"] = True
                    yield True
        if not found:
            yield False

    @owyl.taskmethod
    def startEmotionDetection(self, **kwargs):
        emotion = self.blackboard["emoInput"] + " and " + self.blackboard["affectInput"]

        if self.blackboard["lastemotion"] == emotion:
            yield True
            return

        self.blackboard["lastemotion"] = emotion

        output = "You are very "

        emotion = emotion\
            .replace("anger", "angry")\
            .replace("boredom", "bored")\
            .replace("disgust", "disgusting")\
            .replace("fear", "fearful")\
            .replace("happiness", "happy")\
            .replace("sadness", "sad")\
            .replace("agressiv", "aggressive")\
            .replace("neutral", "cheerful")

        output += emotion
        # For the first around
        if self.blackboard["emotionDetectionStart"]:
            output = "Sure, Let's start the game!"
            self.blackboard["emotionDetectionStart"] = False
        # For the last around
        elif self.blackboard["emotionDetectionEnd"]:
            output = "Okay, " + output + " by the way."
            self.blackboard["isDetectingEmotion"] = False
            self.blackboard["emotionDetectionEnd"] = False

        print output
        # self.itf_talk_pub.publish(output)  # For now
        self.blackboard["robot"].say(output)

        # time.sleep(2000)

        self.blackboard["emoInput"] = ""
        self.blackboard["affectInput"] = ""
        self.blackboard["audioInput"] = ""
        yield True

    @owyl.taskmethod
    def isNotStopObjRecognition(self, **kwargs):
        if not self.blackboard["isRecognizingObject"]:
            yield True
            return

        audioInput = self.blackboard[kwargs["key"]]
        for (key, keywords) in self.blackboard["stopObjRecognition"].iteritems():
            for word in keywords:
                if audioInput.find(word) > -1:
                    self.blackboard["objectRecognitionEnd"] = True
        yield True

    @owyl.taskmethod
    def isObjRecognition(self, **kwargs):
        if self.blackboard["isRecognizingObject"]:
            yield True
            return

        audioInput = self.blackboard[kwargs["key"]]
        found = False
        for (key, keywords) in self.blackboard["playObjRecognition"].iteritems():
            for word in keywords:
                if audioInput.find(word) > -1:
                    found = True
                    self.blackboard["isRecognizingObject"] = True
                    self.blackboard["objectRecognitionStart"] = True
                    yield True
        if not found:
            yield False

    @owyl.taskmethod
    def startObjRecognition(self, **kwargs):
        output = "This is a " + self.blackboard["objectRecognized"]

        # For the first around
        if self.blackboard["objectRecognitionStart"]:
            output = "Sure, Let's start the game!"
            self.blackboard["objectRecognitionStart"] = False
        # For the last around
        elif self.blackboard["objectRecognitionEnd"]:
            output = "Okay, " + output + " by the way."
            self.blackboard["isRecognizingObject"] = False
            self.blackboard["objectRecognitionEnd"] = False

        print output
        # self.itf_talk_pub.publish(output)  # For now
        self.blackboard["robot"].say(output)

        self.blackboard["objectRecognized"] = ""
        self.blackboard["audioInput"] = ""
        yield True

    @owyl.taskmethod
    def isObjInput(self, **kwargs):
        if not self.blackboard["frontObjectRecognized"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoObjInput(self, **kwargs):
        if self.blackboard["frontObjectRecognized"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isVariable(self, **kwargs):
        yield self.blackboard[kwargs["var"]] == kwargs["value"]

    @owyl.taskmethod
    def setVariable(self, **kwargs):
        print "From " + kwargs["var"] + " to " + kwargs["value"]
        self.blackboard[kwargs["var"]] = self.blackboard[kwargs["value"]]
        yield True

    @owyl.taskmethod
    def isSalientTarget(self, **kwargs):
        if len(self.blackboard["frontSaliencyTarget"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isFaceTarget(self, **kwargs):
        if len(self.blackboard["frontFaceTarget"]) > 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoFaceTarget(self, **kwargs):
        if len(self.blackboard["frontFaceTarget"]) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoSalientTarget(self, **kwargs):
        if len(self.blackboard["frontSaliencyTarget"]) == 0:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isAudioInput(self, **kwargs):
        if not self.blackboard["frontAudioInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoAudioInput(self, **kwargs):
        if self.blackboard["frontAudioInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNotSpeaking(self, **kwargs):
        if not self.blackboard["frontSpeechActive"]:
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isRosInput(self, **kwargs):
        if not self.blackboard["frontRosInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoRosInput(self, **kwargs):
        if self.blackboard["frontRosInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isEmotionInput(self, **kwargs):
        if not self.blackboard["frontEmoInput"] == "" and not self.blackboard["frontAffectInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def isNoEmotionInput(self, **kwargs):
        if self.blackboard["frontEmoInput"] == "" and self.blackboard["frontAffectInput"] == "":
            yield True
        else:
            yield False

    @owyl.taskmethod
    def dance(self, **kwargs):
        # self.blackboard["robot"].gesture(Gesture.Dance)
        yield True

if __name__ == "__main__":
    # tree_name = "BasicZenoTree"
    tree_name = "BasicZoidSteinTree"
    tree = Tree(tree_name)
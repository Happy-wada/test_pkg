#!/usr/bin/env python
# -*- coding utf-8 -*-

import rospy 
import smach
import smach_ros
import sys
import actionlib
from happymimi_recognition_msgs.srv import RecognitionList
from enter_room.srv import EnterRoom
from happymimi_msgs.srv import StrTrg
from happymimi_navigation.srv import NaviLocation
from happymimi_manipulation_msgs.srv import RecognitionToGrasping, RecognitionToGraspingRequest
from happymimi_voice_msgs.srv import *
from std_msgs.msg import String, Float64
import roslib
file_path = roslib.packages.get_pkg_dir('happymimi_telop') + '/src/'
from base_control import BaseControl

class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['pass'])
        self.enter_srv = rospy.ServiceProxy('enter_room_server', EnterRoom)
        self.tts_srv = rospy.ServiceProxy('/tts' , TTS)

    def execute(self, userdaa):
        rospy.loginfo("Start Enter Room")
        self.tts_srv("Start Enter Room")
        self.enter_srv(distance=0.8, velocity=0.2)
        return 'pass'

class MoveAndPick(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['success','failed'],
                            input_keys=['object_name_in'],
                            output_keys=['object_name_out'])
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        self.arm_srv = rospy.ServiceProxy('servo/arm', StrTrg)
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.recog_srv = rospy.ServiceProxy('/recognition/list' , RecognitionList)
        
        self.bc = BaseControl()
        self.rotate = 90
        self.recog_result = []

    def execute(self, userdata):
        self.navi_srv('storage')
        sm_name = userdata.object_name_in
        rospy.sleep(0.5)
        grasp_count = 0
        rospy.loginfo('pick')

        self.head_pub.publish(25.0)
        rospy.sleep(2.0)
        self.recog_result = self.recog_srv('cup','left')
        rospy.sleep(1.0)
        print self.recog_result.object_list
        grasp_result = self.grasp_srv(RecognitionToGraspingRequest(target_name='any')).result
        
        if not self.recog_result.object_list:
            pass
        
        else:
            sm_name = 'cup'
            userdata.object_name_out = sm_name
        
        print sm_name
        rospy.sleep(0.5)
        self.bc.rotateAngle(90, 0.4)

        if self.grasp_result  == True:
            return 'success'
        
        else:
            return 'failed'

class MoveAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['completed'],
                            input_keys=['object_name_in'])
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.current_pub = rospy.Publisher('/current_location', String, queue_size = 1)
        self.bc = BaseControl()
        self.recognition_srv = rospy.ServiceProxy('/recognition/list', RecognitionList)

    def execute(self, userdata):
        print userdata.object_name_in
        if userdata.object_name_in in == 'NULL':
            self.navi_srv('Tall table')
            self.current_pub.publish('table')
            self.arm_srv('place')
            self.bc.rotateAngle(180, 0.4)

        else:
            self.navi_srv('box1')
            self.current_pub.publish('couch')
            self_arm_srv('place')
            self.bc.rotateAngle(180, 0.4)

        return 'completed'

class AvoidThat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_WDYS'])
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)

    def execute(self, userdata):
        print('AvoidThat')
        self.navi_srv('operator')
        rospy.sleep(0.5)
        
        return 'to_WDYS'

class PersonSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found'])
        self.tts_srv = rospy.ServiceProxy('/tts', TTS)

    def execute(self, userdata):
        print 'search'

        #if result == True:
        #    m6Control(0.4)
        #    self.tts_srv('I found the Questioner')

        #else:
        #    speak('Please come in front of me')
        #    rospy.sleep(5.0)
        #    self.tts_srv('Thank you')

        return 'found'

class QuestionResponse(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['completed'],
                            input_keys=['success_count_in', 'start_time_out'],
                            output_keys=['success_count_out', 'start_time_out'])
        self.tts_srv = rospy.ServiceProxy('/tts', StrTrg)
        self.WDYS = rospy.ServiceProxy('/bf/conversation_srvserver', WhatDidYouSay)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        self.cnt = 0
        self.bc = BaseControl()

    def execute(self, userdata):
        print "Are you ready?"
        self.tts_srv('Are you ready?')
        if self.yesno_srv().result == True:
            for i in rang(3):
                self.tts_srv('Talk to me.')
                result = self.WDYS().result
                if result == True:
                    cnt += 1
        self.bc.rotateAngle(90, 0.4)
        
        return 'completed'
        
class ExitRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_finish'])
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.tts_srv = rospy.ServiceProxy('/tts', StrTrg)

    def execute(self, userdata):
        print 'exit'
        self.tts_srv('Go to the entrance')
        self.navi_srv('entrance')
        rospy.sleep(0.5)
        self.tts_srv('Finish basic functionalities')
        self.tts_srv('Thnk you very match')
        return 'to_finish'

if __name__ == '__main__':
    rospy.init_node('test_pkg')
    sm_top = smach.StateMachine(outcomes = ['finish_sm'])
    sm_top.userdata.cmd_count = 1
    sm_top.userdata.sm_name = 'NULL'
    with sm_top:
        smach.StateMachine.add('ENTER', EnterRoom(), transitions = {'pass':'MoveAndPick'})

        smach.StateMachine.add('MoveAndPick', MoveAndPick(), 
                        transitions={'success':'MoveAndPlace',
                                     'failed':'AvoidThat'},
                        remapping={'object_name_out':'sm_name',
                                   'object_name_in':'sm_name'})

        smach.StateMachine.add('MoveAndPlace', MoveAndPlace(),
                        transitions={'completed':'AvoidThat'},
                        remapping={'object_name_in':'sm_name'})


        smach.StateMachine.add('AvoidThat',AvoidThat(), transitions={'to_WDYS':'PersonSearch'})

        smach.StateMachine.add('PersonSearch', PersonSearch(), transitions={'found':'QuestionResponse'})
        smach.StateMachine.add('QuestionResponse', QuestionResponse(),
                        transitions={'completed':'ExitRoom'},
                        remapping={'success_count_in':'sm_success',
                                   'success_count_out':'sm_success',
                                   'start_time_in':'sm_time',
                                   'start_time_out':'sm_time'})

        smach.StateMachine.add('ExitRoom', ExitRoom(), transitions={'to_finish':'finish_sm'})

    outcome = sm_top.execute()

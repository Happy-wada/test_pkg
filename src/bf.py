#!/usr/bin/env python
# -*- coding utf-8 -*-

import rospy 
#import std_msgs.msg import String
import smach
import smach_ros
#import time 
#import sys
#import actionlib
#from happymimi_voice_msgs.srv import TTS,
#from enter_room.srv import EnterRoom
#from happymimi_navigation.srv import NaviLocation
#from happymimi_msgs.srv import StrTrg
#from happymimi_manipulation_msgs.srv import RecognitionToGrasping, RecognitionToGraspingRequest
#sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
#from voice_common_pkg.srv import WhatDidYouSay

class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['pass'])
        #self.enter_srv = rospy.ServiceProxy('enter_room_server', EnterRoom)

    def execute(self, userdaa):
        rospy.loginfo("Start Enter Room")
        #self.enter_srv(distance=0.8, velocity=0.2)
        #speak('Start  test')
        return 'pass'


class MoveAndPick(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['success','failed'],
                            input_keys=['object_name_in'],
                            output_keys=['object_name_out'])

        #self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        #self.arm_srv = rospy.ServiceProxy('servo/arm', StrTrg)
        #self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        
        #self.rotate = 90


        

    def execute(self, userdata):
        #self.navi_srv('storage')
        #rospy.sleep(0.5)
        #grasp_count = 0
        rospy.loginfo('pick')

        #while not rospy.is_shutdown():
            #rospy.sleep(0.3)
            #self.grasp_result = self.grasp_srv(RecognitionToGraspingRequest(target_name='any')).result
        


        #rospy.wait_for_service('/object/rcognize')
        #recog = rospy.ServiceProxy('object/recognize', RecognizeCount)
        #res = recog('any')

        #if len(res.data) >= 2:
        #    object_name = res.data[1]

        #elif len(res.data) == 1:
        #    object_name = res.data[0]

        #else:
        #    object_name = 'any'
        #userdata.object_name_out = target_name
        #self.pub_location.pubish('table')

        #result = self.grab(object_name).result
        #if self.grasp_result  == True:
        return 'success'
        #else:
            #return 'failed'

class MoveAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['completed'],
                            input_keys=['object_name_in'])
        #self.object_list = ['cup', 'bottle', 'snack', 'dish', 'chips', 'bag', 'toy', 'smartphone', 'book', 'pen',]

    def execute(self, userdata):
        print 'place'
        #if userdata.object_name_in in self.object_list:
            #self.navi_srv('Tall table')
            #location_list = searchLocationName('')
            #self.pub_location.publish('')
        #else:
            #location_list = searchLocationName('')
            #self.pub_location.publish('')

        #navigationAC(location_list)
        #self.arm_srv('place')
        return 'completed'

class AvoidThat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_WDYS'])
        
    def execute(self, userdata):
        print('AvoidThat')
        #location_list = searchLocationName('operator')
        #navigationAC(location_list)
        #self.navi_srv('operator')
        #rospy.sleep(0.5)
        return 'to_WDYS'


class PersonSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found'])

        #self.flag = 'failed'

    def execute(self, userdata):
        print 'search'

        #if result == True:
        #    m6Control(0.4)
        #    speak('I found the Questioner')

        #else:
        #    speak('Please come in front of me')
        #    rospy.sleep(5.0)
        #    speak('Thank you')
        return 'found'

class QuestionResponse(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['completed'],
                            input_keys=['success_count_in', 'start_time_out'],
                            output_keys=['success_count_out', 'start_time_out'])
        #self.WDYS = rospy.ServiceProxy('/bf/conversation_srvserver', WhatDidYouSay)
        #cnt = 0

    def execute(self, userdata):
        print "Are you ready?"
        #speak('Are you ready?')
        #if str(stt_pub(short_str=True).result_str) == 'yes':
        #    for i in range(5):
        #        tts_pub('Talk to me.')
        #        result = self.WDYS().result
        #        if result == True:
        #            cnt += 1

        return 'completed'

        
        
class ExitRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_finish'])

    def execute(self, userdata):
        print 'exit'
        #speak('Go to the exit')
        #location_list = searchLocationName('entrance')
        #navigationAC(location_list)
        #self.navi_srv('entrance')
        #rospy.sleep(0.5)

        return 'to_finish'

if __name__ == '__main__':
    rospy.init_node('test_pkg')
    sm_top = smach.StateMachine(outcomes = ['finish_sm'])
    sm_top.userdata.cmd_count = 1
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

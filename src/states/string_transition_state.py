#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach_ros
import smach
from std_msgs.msg import String
from std_msgs.msg import Int16

class StringTransitionState(smach.State):
    def __init__(self, topic, outcomes=[], input_keys=[], output_keys=[]):
        self._topic = topic
        smach.State.__init__(self, outcomes, input_keys, output_keys)

    def execute(self, userdata):
        #print(self._topic)
        while rospy.is_shutdown() is False:
            userdata.ctrl_type = 1
            userdata.target_outcome = 0

            #print('wait for message')
            trans_tag = rospy.wait_for_message(self._topic,String)
            #print(trans_tag.data)

            if trans_tag.data in self._outcomes:
                return trans_tag.data
            
class IntTransitionStateSim(smach.State):
    def __init__(self, outcomes=[], input_keys=[], output_keys=[]):
        self.count = rospy.Time.now()
        self.ispub = False
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.pub = rospy.Publisher('/ns0/mujoco_ros/mujoco_ros_interface/ctrl_type', Int16)

    def execute(self, userdata):
        while rospy.is_shutdown() is False:
            if self.ispub == False:        
                self.count = rospy.Time.now() 
                self.pub.publish(userdata.ctrl_type)
                self.ispub = True

            if rospy.Time.now() - self.count >= rospy.Duration(userdata.duration):   
                self.count = rospy.Time.now()
                self.ispub = False

                # assign next transition
                trans_tag = String()
                if sorted(list(self._outcomes))[userdata.target_outcome] == 'a_home':
                    userdata.ctrl_type = 1
                    userdata.duration = 4.0

                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'b_home':
                    userdata.ctrl_type = 1
                    userdata.duration = 4.0
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'a_trans_pos':
                    userdata.ctrl_type = 10
                    userdata.duration = 4.0       
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'b_trans_pos':
                    userdata.ctrl_type = 10
                    userdata.duration = 4.0
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'a_recog_pos':
                    userdata.ctrl_type = 11
                    userdata.duration = 4.0
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'a_bottle_approach':
                    userdata.ctrl_type = 12
                    userdata.duration = 4.0
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'a_bottle_pick':
                    userdata.ctrl_type = 13
                    userdata.duration = 2.0                    
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'b_robot_approach':
                    userdata.ctrl_type = 14
                    userdata.duration = 4.0
                
                elif sorted(list(self._outcomes))[userdata.target_outcome] == 'b_bottle_place':
                    userdata.ctrl_type = 15
                    userdata.duration = 2.0
                   
                trans_tag.data = sorted(list(self._outcomes))[userdata.target_outcome]     

                if sorted(list(self._outcomes))[userdata.target_outcome] == 'b_trans_pos':
                    userdata.target_outcome = 1            
           
                return trans_tag.data             

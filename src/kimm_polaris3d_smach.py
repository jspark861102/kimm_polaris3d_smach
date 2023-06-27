#! /usr/bin/python3

from __future__ import print_function

import rospy
from smach import StateMachine
import smach_ros
import smach

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
import numpy as np

from math import pi

from states import StringTransitionState
from states import IntTransitionStateSim

def main():
    rospy.init_node('kimm_polaris3d_smach')
    topic_name = '/ns0/kimm_polaris3d/state_transition'
    # issimulation = rospy.get_param("/issimulation")

    polaris3d_sm = StateMachine(outcomes=['finished','aborted','preempted'])
    polaris3d_sm.userdata.ctrl_type = 1; #home
    polaris3d_sm.userdata.target_outcome = 0; #assign next outcome that will be used    
    polaris3d_sm.userdata.duration = 4.0; #assign next outcome that will be used    

      
    ## defining state machine structure
    with polaris3d_sm:
        StateMachine.add('START',
            StringTransitionState(topic_name, outcomes=['pick_and_place', 'finish'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'pick_and_place':'HOME', 'finish':'finished'})

        StateMachine.add('HOME',
            IntTransitionStateSim(outcomes=['a_trans_pos', 'b_finish'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'a_trans_pos':'TRNAS_POS', 'b_finish':'START'})  
          
        StateMachine.add('TRNAS_POS',
            IntTransitionStateSim(outcomes=['a_recog_pos', 'b_robot_approach'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'a_recog_pos':'RECOG_POS', 'b_robot_approach':'ROBOT_APPROACH'})  
              
        StateMachine.add('RECOG_POS',
            IntTransitionStateSim(outcomes=['a_bottle_approach'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'a_bottle_approach':'BOTTLE_APPROACH'})   
        
        StateMachine.add('BOTTLE_APPROACH',
            IntTransitionStateSim(outcomes=['a_bottle_pick'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'a_bottle_pick':'BOTTLE_PICK'})        
        
        StateMachine.add('BOTTLE_PICK',
            IntTransitionStateSim(outcomes=['b_trans_pos'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'b_trans_pos':'TRNAS_POS'})  
           
        StateMachine.add('ROBOT_APPROACH',
            IntTransitionStateSim(outcomes=['a_dummy', 'b_bottle_place'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'a_dummy':'preempted', 'b_bottle_place':'BOTTLE_PLACE'})   

        StateMachine.add('BOTTLE_PLACE',
            IntTransitionStateSim(outcomes=['a_dummy','b_home'], input_keys=['ctrl_type','target_outcome', 'duration'], output_keys=['ctrl_type','target_outcome', 'duration']), 
            transitions={'a_dummy':'preempted', 'b_home':'HOME'})   


       
    # Run state machine introspection server
    intro_server = smach_ros.IntrospectionServer('kimm_polaris3d', polaris3d_sm,'/POLARIS3D')
    intro_server.start()
    polaris3d_sm.execute()

    rospy.spin()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
    

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from FingerMathURDF import HAND_FingerPosToAngle
print(HAND_FingerPosToAngle(0, 0))


node_name = 'rohand_urdf_node'

THUMB_ID = 0
INDEX_FINGER_ID = 1
MIDDLE_FINGER_ID = 2
RING_FINGER_ID = 3
LITTLE_FINGER_ID = 4
THUMB_ROOT_ID = 5

JOINTS_NAME = [
    ['th_proximal_link','th_slider_link','th_connecting_link','th_distal_link'],
    ['if_slider_link','if_slider_abpart_link','if_proximal_link', 'if_distal_link','if_connecting_link'],
    ['mf_slider_link','mf_slider_abpart_link','mf_proximal_link', 'mf_distal_link','mf_connecting_link'],
    ['rf_slider_link','rf_slider_abpart_link','rf_proximal_link', 'rf_distal_link','rf_connecting_link'],
    ['lf_slider_link','lf_slider_abpart_link','lf_proximal_link', 'lf_distal_link','lf_connecting_link'],
    ['th_root_link']
]

class ROHandURDFNode:

    def __init__(self):
        rospy.init_node(node_name,anonymous=True)
        
        rospy.loginfo(f'++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
        rospy.loginfo(f'++++++++++++++++++Start to publish joint states+++++++++++++++++++')
        rospy.loginfo(f'++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')

        self.joint_states_publisher = rospy.Publisher('joint_states', JointState, queue_size = 10)
        
        self.joint_states_sub = rospy.Subscriber(
            'joint_states', 
            JointState,
            self._joint_states_callback,
            queue_size=10
        )

    def cal_joint_angle(self, finger_id, position, msg):
            try:
                joint_angle = HAND_FingerPosToAngle(finger_id, position)
            except NameError:
                rospy.logerr("函数未定义！请检查拼写或导入")
            msg.name.extend(JOINTS_NAME[finger_id])

            if(finger_id == THUMB_ID):
                msg.position.extend([joint_angle[0], position, joint_angle[1], joint_angle[2]])
            elif(finger_id == THUMB_ROOT_ID):
                msg.position.append(position)
            else:
                msg.position.append(position)
                msg.position.extend(joint_angle)
       


    def _joint_states_callback(self, msg):
        rotation_msg = JointState()
        rotation_msg.header = msg.header
        rotation_msg.name = []
        rotation_msg.position = []

        for i, name in enumerate(msg.name):
          
            if name == 'if_slider_link':
                position = msg.position[i]
                self.cal_joint_angle(INDEX_FINGER_ID, position, rotation_msg)
                
            if name == 'mf_slider_link':
                position = msg.position[i]
                self.cal_joint_angle(MIDDLE_FINGER_ID, position, rotation_msg)
     
            if name == 'rf_slider_link':
                position = msg.position[i]
                self.cal_joint_angle(RING_FINGER_ID, position, rotation_msg)

            if name == 'lf_slider_link':
                position = msg.position[i]
                self.cal_joint_angle(LITTLE_FINGER_ID, position, rotation_msg)

            if name == 'th_slider_link':
                position = msg.position[i]
                self.cal_joint_angle(THUMB_ID, position, rotation_msg)

            if name == 'th_root_link':
                position = msg.position[i]
                self.cal_joint_angle(THUMB_ROOT_ID, position, rotation_msg)

        # publish message
        self.joint_states_publisher.publish(rotation_msg)
        rospy.loginfo(f'Published rotation angle: {rotation_msg.position}\n')
        

def main():
    node = ROHandURDFNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

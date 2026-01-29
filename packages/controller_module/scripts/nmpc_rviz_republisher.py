#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from controller_module.msg import Telemetry


class NMPCRvizRepublisher:

    def __init__(self) -> None:
        rospy.init_node("NMPCRvizRepublisher")    
        self.pub_predicted_trajectory = rospy.Publisher('control_manager/controller_module/Predicted_trajectory',
                                 PoseArray, queue_size=1000)
        self.pub_reference_trajectory = rospy.Publisher('control_manager/controller_module/Reference_trajectory',
                                 PoseArray, queue_size=1000)
        self.pub_initial_state = rospy.Publisher('control_manager/controller_module/Initial_state',
                                 PoseStamped, queue_size=1000)
        self.sub_telemetry = rospy.Subscriber('control_manager/controller_module/Telemetry',
                                 Telemetry,self.callback_prediction)


    def run(self):
        rospy.spin();
      
    def callback_prediction(self,msg):
        predicted_drone_states = msg.predicted_drone_states
        prediction_pose_array = PoseArray()
        prediction_pose_array.header = msg.header
        for i in range(len(predicted_drone_states)):
            prediction_pose_array.poses.append(predicted_drone_states[i].pose)
        self.pub_predicted_trajectory.publish(prediction_pose_array)
        
        reference_trajectory = msg.reference_trajectory
        reference_pose_array = PoseArray()
        reference_pose_array.header = msg.header
        for i in range(len(reference_trajectory)):
            reference_pose_array.poses.append(reference_trajectory[i].pose)
        self.pub_reference_trajectory.publish(reference_pose_array)

        initial_state_pose_stamped = PoseStamped()
        initial_state_pose_stamped.header = msg.header
        initial_state_pose_stamped.pose = msg.initial_state.pose
        self.pub_initial_state.publish(initial_state_pose_stamped)


if __name__ == '__main__':
    try:
        republisher = NMPCRvizRepublisher()
        republisher.run()
    except rospy.ROSInterruptException:
        pass

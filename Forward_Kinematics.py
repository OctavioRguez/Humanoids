#!/usr/bin/env python
import rospy
import pinocchio
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class ForwardKinematics:
    # Initialization
    def __init__(self):
	# Namefile
        self.__urdf_filename = '/home/humanoid/reemc_public_ws/src/whole_body_state_msgs/urdf/reemc_full_ft_hey5.urdf'
	self.__free_flyer = pinocchio.JointModelFreeFlyer() # Create object for free flyer

	self._flag = False # Flag to start calculating the forward kinematics

    def _baseCallback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
	if (self._flag):
	    self.__q[0] = position.x
	    self.__q[1] = position.y
	    self.__q[2] = position.z
	    self.__q[3] = orientation.x
	    self.__q[4] = orientation.y
	    self.__q[5] = orientation.z
	    self.__q[6] = orientation.w

    def _jointCallback(self, msg):
        num_joints = len(msg.position)
	if (self._flag):
            self.__q[7 : 7+num_joints] = msg.position

    def _processing(self):
        self.__model = pinocchio.buildModelFromUrdf(self.__urdf_filename, self.__free_flyer) # Get model
        print('Model name: ' + self.__model.name)

        self.__data = self.__model.createData() # Create data
        self.__q = pinocchio.neutral(self.__model) # Get configuration space
        print('q: %s' % self.__q.T)

	self._flag = True # Change flag

    def kinematics(self):
	pinocchio.forwardKinematics(self.__model, self.__data, self.__q) # Calculate forward kinematics

        for name, oMi in zip(self.__model.names, self.__data.oMi): # Print information from the model
            print(("{:<24} : {:.3f} {:.3f} {:.3f}".format(name, *oMi.translation.T)))

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node('Forward_Kinematics')
    rospy.on_shutdown(stop)

    hz = 50 # Frequency (Hz)
    rate = rospy.Rate(hz)

    kinematics = ForwardKinematics() # Kinematics class object

    rospy.Subscriber("/floating_base_pose_simulated", Odometry, kinematics._baseCallback) # Get the floating base
    rospy.Subscriber("/joint_states", JointState, kinematics._jointCallback) # Get the joints states

    print("The Forward Kinematics is Running")

    # Run the node
    while not rospy.is_shutdown():
	# Execute the processing only once
	if (not kinematics._flag):
	    kinematics._processing()
	else:
	    kinematics.kinematics()

	rate.sleep()



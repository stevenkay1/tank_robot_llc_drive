#!/usr/bin/env python

import rospy
import serial
import math

from geometry_msgs.msg import Twist
# from tank_robot_motor_state.msg import Motor_State


class cmd_velocities:

	def __init__(self,max):

		self.lin_x = 0
		self.rot_z = 0
		self.max_value = max

	def setVelocities(self,vel_x,ang_z):

		if vel_x > 1.0:
			vel_x = 1.0
		elif vel_x < -1.0:
			vel_x = -1.0

		if ang_z > 1.0:
			ang_z = 1.0
		elif ang_z < -1.0:
			ang_z = -1.0

		self.lin_x = int(vel_x * self.max_value)
		self.rot_z = int(ang_z * self.max_value)

		#rospy.loginfo("Converted Commands - lin_x - " + str(self.lin_x) + " rot_z - " + str(self.rot_z))



class motor_driver_comms:

	def __init__(self,path,baud_rate):
		self.device_path = path
		self.baud = baud_rate

	def initialise(self):
		try:
			self.ser = serial.Serial(self.device_path, self.baud, timeout = 1)
			
			if self.ser.isOpen():
				rospy.loginfo("Tank Robot Motor Controller Connect\n")
				return 0
		except Exception, e:
			rospy.loginfo(str(e))
			return 1

	def create_command_seq_skid_steer(self,lin,rot):
			
		if lin > 0 and rot == 0:
			M1_dir = 0
			M2_dir = 0
			speed = lin

		elif lin < 0 and rot == 0:
			M1_dir = 1
			M2_dir = 1
			speed = abs(lin)

		elif rot > 0 and lin == 0:
			M1_dir = 0
			M2_dir = 1
			speed = rot

		elif rot < 0 and lin == 0:
			M1_dir = 1
			M2_dir = 0
			speed = abs(rot)

 

		else:
			M1_dir = 0
			M2_dir = 0
			speed = 0


		command = str(str(chr(M1_dir)) + str(chr(M2_dir)) + str(chr(speed)) + str(chr(speed)))

		#rospy.loginfo("m1_dir - " + str(M1_dir))
		#rospy.loginfo("m2_dir - " + str(M2_dir))
		#rospy.loginfo("speed - " + str(speed))

		#rospy.loginfo("Command to send - " + command)

		return command



	def send_motor_commands(self,linear_speed,rot_speed):
		command_seq = self.create_command_seq_skid_steer(linear_speed,rot_speed)
		self.ser.write(command_seq)


	def publish_motor_status(self,ros_node):
		print "hello"


	def terminate():
		self.ser.shutdown()
		rospy.loginfo("Tank Robot Motor Controller Disconnected\n")
		return 0




def callback(data,args):

	robot_vel = args[0]
	robot_comms = args[1]

	#rospy.loginfo(rospy.get_caller_id() + "lin_x - "+str(data.linear.x) + " rot_z - " + str(data.angular.z)+"\n")

	robot_vel.setVelocities(data.linear.x,data.angular.z)

	robot_comms.send_motor_commands(robot_vel.lin_x,robot_vel.rot_z)



def main():


	rospy.init_node('tank_robot_llc_drive', anonymous = True)

	motor_controller_path = rospy.get_param('~controller_path')
	motor_controller_baud = rospy.get_param('~baud')
	max_speed = rospy.get_param('~max_speed')

	# motor_status_pub = rospy.Publisher('motor_status',Motor_State,queue_size=1)




	tank_robot_velocities = cmd_velocities(max_speed)
	tank_robot_motor_comms = motor_driver_comms(motor_controller_path,
		motor_controller_baud)

	tank_robot_motor_comms.initialise()

	rospy.Subscriber("cmd_vel", Twist, callback,(tank_robot_velocities,tank_robot_motor_comms))

	rospy.spin()


if __name__ == "__main__":
	main()

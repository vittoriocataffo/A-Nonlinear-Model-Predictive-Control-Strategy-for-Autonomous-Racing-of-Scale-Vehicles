#!/usr/bin/env python3
from __future__ import division

import sys

"""
WARNING: Update this path with the folder containing the built version of the
optimization problem.
"""
sys.path.insert(1, "/home/gsilano/Pictures/nmpc_ws/OBS_PANOC_DYNAMIC_MOTOR_MODEL/dynamic_my_optimizer/dynamic_racing_target_point")
import dynamic_racing_target_point
solver = dynamic_racing_target_point.solver()

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time
import casadi.casadi as cs
import numpy as np
import math
import csv

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


def find_the_center_line(X_fut,Y_fut,center_x,center_y):
    """
    Find the center line of the track
    """
    dist_x = np.zeros(len(center_x))
    dist_y = np.zeros(len(center_x))
    r = np.zeros((N,len(center_x)))
    center_x_proj = np.zeros(N)
    center_y_proj = np.zeros(N)

    for j in range(len(X_fut)):
        dist_x = (X_fut[j] - center_x)**2
        dist_y = (Y_fut[j] - center_y)**2
        r = dist_x+dist_y
        x = np.argmin(r)
        center_x_proj[j] = center_x[x]
        center_y_proj[j] = center_y[x]

    return center_x_proj, center_y_proj


def perception_target_point(X_odom,Y_odom,center_x,center_y,a):

	center_x = np.concatenate((center_x, center_x))
	center_y = np.concatenate((center_y, center_y))
	dist_x = np.empty(len(center_x))
	dist_y = np.empty(len(center_x))
	r = np.empty(len(center_x))

	dist_x = (X_odom - center_x)**2
	dist_y = (Y_odom - center_y)**2
	r = dist_x+dist_y;

	x = np.argmin(r)
	target_point_x = center_x[x+a]
	target_point_y = center_y[x+a]

	return target_point_x, target_point_y

############################################################################################################################################
############################################################################################################################################
############################################################################################################################################
"""
WARNING: Update these paths with the name of the folder containing the description of the track,
i.e., where the car will race.
"""
csv_file = np.genfromtxt('/home/gsilano/Desktop/DATA/Map_track1/center_x_track1.csv',
                          delimiter=',', dtype=float)
center_x = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/gsilano/Desktop/DATA/Map_track1/center_y_track1.csv',
                          delimiter=',', dtype=float)
center_y = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/gsilano/Desktop/DATA/Map_track1/bound_x1_track1.csv',
                          delimiter=',', dtype=float)
bound_x1 = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/gsilano/Desktop/DATA/Map_track1/bound_y1_track1.csv',
                          delimiter=',', dtype=float)
bound_y1 = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/gsilano/Desktop/DATA/Map_track1/bound_x2_track1.csv',
                          delimiter=',', dtype=float)
bound_x2 = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/gsilano/Desktop/DATA/Map_track1/bound_y2_track1.csv',
                          delimiter=',', dtype=float)
bound_y2 = csv_file[:].tolist()

###1 - XY coordinates of the obstacles
obs_x = 17.5
obs_y = -6
obs_x_2 = 30
obs_y_2 = -12.5

###2 - XY coordinates of the obstacles
#obs_x = -30
#obs_y = 7
#obs_x_2 = -3
#obs_y_2 = 9

###3 - XY coordinates of the obstacles
#obs_x = -8
#obs_y = 8.5
#obs_x_2 = -11.5
#obs_y_2 = 11
############################################################################################################################################
############################################################################################################################################
############################################################################################################################################
"""
WARNING: Update this path with the place where you want to save the csv files with the data
retrieved from the F1/10 simulation.
"""
f = open('/home/gsilano/Desktop/DATA/race_DATA.csv', 'w')
writer = csv.writer(f)

rospy.init_node('my_mpc_node',anonymous = True)

LRW_topic   = '/car_1/left_rear_wheel_velocity_controller/command'
RRW_topic   = '/car_1/right_rear_wheel_velocity_controller/command'
LFW_topic   = '/car_1/left_front_wheel_velocity_controller/command'
RFW_topic   = '/car_1/right_front_wheel_velocity_controller/command'
LSH_topic   = '/car_1/left_steering_hinge_position_controller/command'
RSH_topic   = '/car_1/right_steering_hinge_position_controller/command'

pub_vel_LRW = rospy.Publisher(LRW_topic, Float64, queue_size = 1)
pub_vel_RRW = rospy.Publisher(RRW_topic, Float64, queue_size = 1)
pub_vel_LFW = rospy.Publisher(LFW_topic, Float64, queue_size = 1)
pub_vel_RFW = rospy.Publisher(RFW_topic, Float64, queue_size = 1)
pub_pos_LSH = rospy.Publisher(LSH_topic, Float64, queue_size = 1)
pub_pos_RSH = rospy.Publisher(RSH_topic, Float64, queue_size = 1)

steering_angle = Float64()
velocity = Float64()

rate = rospy.Rate(30)

# Car's paramters
N = 50
T = 0.033

car_width = 0.25;
car_length = 0.4;
lr = 0.147;
lf = 0.178;
m  = 5.6922;
Iz  = 0.204;
#df= 199.622
#dr= 191.2118
#cf= 0.057849
#cr= 0.11159
#bf= 9.2567
#br= 17.7464
#Cm1= 20
#Cm2= 1.3856e-07
#Cm3= 3.9901
#Cm4= 0.66633
df= 134.585
dr= 159.9198
cf= 0.085915
cr= 0.13364
bf= 9.2421
br= 17.7164
Cm1= 20
Cm2= 6.9281e-07
Cm3= 3.9901
Cm4= 0.66633
n_states = 6
n_controls = 2

mpciter = 0
u_cl1 = 0
u_cl2 = 0
xx1 = np.empty(N+1)
xx2 = np.empty(N+1)
xx3 = np.empty(N+1)
xx4 = np.empty(N+1)
xx5 = np.empty(N+1)
xx6 = np.empty(N+1)
x0 = [0, 0, 0, 1, 0, 0]		# initial conditions
xx_hist = []
xy_hist = []
xtheta_hist = []		# history of the states
vx_hist = []
vy_hist = []
omega_hist = []
ucl = []
guess = [0.0]*(2*N)
theta2unwrap = []


#################################################################################################################################
#################################################################################################################################
#################################################################################################################################

def callback(data):
	global mpciter
	global u_cl1, u_cl2, xx1, xx2, xx3, xx4, xx5, xx6, x0, xx_hist, xy_hist, xtheta_hist, ucl, guess

	now_rostime = rospy.get_rostime()
	rospy.loginfo("Current time %f", now_rostime.secs)

	Pose = data.pose.pose
	x_odom = Pose.position.x
	y_odom = Pose.position.y
	quaternion = (Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w)
	euler = euler_from_quaternion(Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w)
	theta_odom = euler[2]
	theta2unwrap.append(theta_odom)
	thetaunwrapped = np.unwrap(theta2unwrap)
	x0[0] = x_odom
	x0[1] = y_odom
	x0[2] = thetaunwrapped[-1]
	Twist = data.twist.twist
	if mpciter<15:
		x0[3] = 3
	else:
		x0[3] = Twist.linear.x * math.cos(x0[2]) + Twist.linear.y * math.sin(x0[2])
	x0[4] = Twist.linear.y * math.cos(x0[2]) - Twist.linear.x * math.sin(x0[2])
	x0[5] = Twist.angular.z
	x03 = Twist.linear.x * math.cos(x0[2]) + Twist.linear.y * math.sin(x0[2])

	if mpciter < 1:
		proj_center = find_the_center_line(np.linspace(0,1,N),np.zeros(N),center_x,center_y)
		proj_center_X = proj_center[0]
		proj_center_Y = proj_center[1]
	else:
		proj_center = find_the_center_line(xx1[1:N+1],xx2[1:N+1],center_x,center_y)
		proj_center_X = proj_center[0]
		proj_center_Y = proj_center[1]

	target_point = perception_target_point(x0[0],x0[1],center_x,center_y,90)


	parameter = []
	for i in range(n_states):
		parameter.append(x0[i])
	# preU
	parameter.append(u_cl1)
	parameter.append(u_cl2)
	# target point
	parameter.append(target_point[0])
	parameter.append(target_point[1])
	# center line projection
	for i in range(N):
		parameter.append(proj_center_X[i])
		parameter.append(proj_center_Y[i])
	parameter.append(obs_x)
	parameter.append(obs_y)
	parameter.append(obs_x_2)
	parameter.append(obs_y_2)

	now = time.time()
	result = solver.run(p=[parameter[i] for i in range(n_states + n_controls + 2 + 2*N + 4)],initial_guess=[guess[i] for i in range (n_controls*N)])
	now1 = time.time()
	elapsed = now1-now
	print(elapsed)

	u_star = np.full(n_controls*N,result.solution)
	guess = u_star

	u_cl1 = u_star[0]
	u_cl2 = u_star[1]

	xx1[0] = x0[0]
	xx2[0] = x0[1]
	xx3[0] = x0[2]
	xx4[0] = x0[3]
	xx5[0] = x0[4]
	xx6[0] = x0[5]

	for i in range(N):
		xx1[i+1] = xx1[i] + T* (xx4[i]*math.cos(xx3[i])-xx5[i]*math.sin(xx3[i]))
		xx2[i+1] = xx2[i] + T* (xx4[i]*math.sin(xx3[i])+xx5[i]*math.cos(xx3[i]))
		xx3[i+1] = xx3[i] + T* (xx6[i])
		xx4[i+1] = xx4[i] + T* ((1/m)*( (Cm1-Cm2*xx4[i])*guess[2*i]- Cm4*xx4[i]**2 -Cm3 + ((Cm1-Cm2*xx4[i])*guess[2*i]- Cm4*xx4[i]**2 -Cm3)*math.cos(guess[2*i+1]) + m*xx5[i]*xx6[i] - df*math.sin(cf*math.atan(bf*(- math.atan((xx5[i] + lf*xx6[i])/xx4[i]) + guess[2*i+1])))*math.sin(guess[2*i+1]) ))
		xx5[i+1] = xx5[i] + T* ((1/m)*(((Cm1-Cm2*xx4[i])*guess[2*i]- Cm4*xx4[i]**2 -Cm3)*math.sin(guess[2*i+1])-m*xx4[i]*xx6[i] + (df*math.sin(cf*math.atan(bf*(- math.atan((xx5[i] + lf*xx6[i])/xx4[i]) + guess[2*i+1])))*math.cos(guess[2*i+1]) + dr*math.sin(cr*math.atan(br*(- math.atan((xx5[i] - lr*xx6[i])/xx4[i]))))) ))
		xx6[i+1] = xx6[i] + T* ((1/Iz)*( lf*((Cm1-Cm2*xx4[i])*guess[2*i]- Cm4*xx4[i]**2 -Cm3)*math.cos(guess[2*i+1]) + lf*df*math.sin(cf*math.atan(bf*(- math.atan((xx5[i] + lf*xx6[i])/xx4[i]) + guess[2*i+1])))*math.cos(guess[2*i+1])-lr*dr*math.sin(cr*math.atan(br*(- math.atan((xx5[i] - lr*xx6[i])/xx4[i]))))))

	velocity.data = u_cl1*100
	steering_angle.data = u_cl2



	pub_pos_LSH.publish(steering_angle)
	pub_pos_RSH.publish(steering_angle)
	pub_vel_LFW.publish(velocity)
	pub_vel_RFW.publish(velocity)
	pub_vel_LRW.publish(velocity)
	pub_vel_RRW.publish(velocity)

	row = [x0[0], x0[1], x0[2], x03, x0[4], x0[5],elapsed,u_cl1,u_cl2,now_rostime]
	writer.writerow(row)

	mpciter = mpciter+1
	rate.sleep()



if __name__ == '__main__':
	print("my node started")
	rospy.Subscriber("/car_1/ground_truth",Odometry,callback,queue_size=1)
	#rospy.Subscriber("/car_1/base/odom",Odometry,callback,queue_size=1)
	rospy.spin()

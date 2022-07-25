import sys
sys.path.insert(1, "/home/vittorix/Scrivania/PANOC DYNAMIC MOTOR MODEL/dynamic_my_optimizer/dynamic_racing_target_point")
import dynamic_racing_target_point
solver = dynamic_racing_target_point.solver()

import time
import math
import numpy as np
import matplotlib.pyplot as plt
import csv

def find_the_center_line(X_fut,Y_fut,center_x,center_y):    
 
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


f = open('/home/vittorix/Scrivania/DATA_PANOC_2compare.csv', 'w')
writer = csv.writer(f)

csv_file = np.genfromtxt('/home/vittorix/Scrivania/Map_track3/center_x_track3.csv', 
                          delimiter=',', dtype=float)
center_x = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/vittorix/Scrivania/Map_track3/center_y_track3.csv', 
                          delimiter=',', dtype=float)
center_y = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/vittorix/Scrivania/Map_track3/bound_x1_track3.csv', 
                          delimiter=',', dtype=float)
bound_x1 = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/vittorix/Scrivania/Map_track3/bound_y1_track3.csv', 
                          delimiter=',', dtype=float)
bound_y1 = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/vittorix/Scrivania/Map_track3/bound_x2_track3.csv', 
                          delimiter=',', dtype=float)
bound_x2 = csv_file[:].tolist()
csv_file = np.genfromtxt('/home/vittorix/Scrivania/Map_track3/bound_y2_track3.csv', 
                          delimiter=',', dtype=float)
bound_y2 = csv_file[:].tolist()

T = 0.033
N = 50

car_width = 0.281;
car_length = 0.535;
lr = 0.160; 
lf = 0.164;
m  = 4;
Iz  = 0.1217;
cf = 0.0867;
cr = 0.1632;
mu = 0.1;
df = 42.5255;
dr = 161.585;
bf = 29.495;
br = 26.9705;
Cm1 = 1.8073
Cm2 = -0.2511
n_states = 6
n_controls = 2

x0 = [0, 0, 0, 1, 0, 0]      # initial conditions

xx_hist = []
xy_hist = []
xtheta_hist = []        # history of the states
xvx_hist = [] 
xvy_hist = []
xomega_hist = []
elapsed_hist = []

ucl = []    # history of the control inputs
u_cl1 = 0
u_cl2 = 0

xx1 = np.empty(N+1)
xx2 = np.empty(N+1)
xx3 = np.empty(N+1)
xx4 = np.empty(N+1)
xx5 = np.empty(N+1)
xx6 = np.empty(N+1)

xx_hist.append(x0[0])
xy_hist.append(x0[1])
xtheta_hist.append(x0[2])
xvx_hist.append(x0[3])
xvy_hist.append(x0[4])
xomega_hist.append(x0[5])
ucl.append(0)
ucl.append(0)

guess = [0.0] * (2*N)

total_time = 0

mpciter = 0

sim_steps = 2000

# to run GUI event loop
plt.ion()
# here we are creating sub plots
figure, ax = plt.subplots()
line1, = ax.plot(np.array([1, 1, 1, 1, 1]), np.array([1, 1, 1, 1, 1]), 'r')
line2, = ax.plot(xx1,xx2)
line3, = ax.plot(center_x,center_y,'k--')
line4, = ax.plot(x0[0],x0[0],'g*')
#line5, = ax.plot(xx1[1:N+1],xx2[1:N+1],'bo')
line6, = ax.plot(bound_x1,bound_y1,'k')
line7, = ax.plot(bound_x2,bound_y2,'k')
plt.ylim((-50,50))
plt.xlim((-50,50))

while mpciter<sim_steps:


    if mpciter < 1:
        proj_center = find_the_center_line(np.linspace(0,1,N),np.zeros(N),center_x,center_y)
        proj_center_X = proj_center[0]
        proj_center_Y = proj_center[1]
    else:
        proj_center = find_the_center_line(xx1[1:N+1],xx2[1:N+1],center_x,center_y)
        proj_center_X = proj_center[0]
        proj_center_Y = proj_center[1]



    target_point = perception_target_point(x0[0],x0[1],center_x,center_y,80)

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
    



    now = time.time()
    
    result = solver.run(p=[parameter[i] for i in range(n_states + n_controls + 2 + 2*N)],initial_guess=[guess[i] for i in range (n_controls*N)])
    now1 = time.time()
    elapsed = now1 - now
    elapsed_hist.append(elapsed)
    total_time = total_time + elapsed

    print("elapsed time",elapsed)

    u_star = np.full(n_controls*N,result.solution)
    guess = u_star

    u_cl1 = u_star[0]
    u_cl2 = u_star[1]

    car11 = x0[0] + math.cos(x0[2])*car_length/2 + math.sin(x0[2])*car_width/2
    car12 = x0[1] + math.sin(x0[2])*car_length/2 - math.cos(x0[2])*car_width/2
    car21 = x0[0] + math.cos(x0[2])*car_length/2 - math.sin(x0[2])*car_width/2
    car22 = x0[1] + math.sin(x0[2])*car_length/2 + math.cos(x0[2])*car_width/2
    car31 = x0[0] - math.cos(x0[2])*car_length/2 + math.sin(x0[2])*car_width/2
    car32 = x0[1] - math.sin(x0[2])*car_length/2 - math.cos(x0[2])*car_width/2
    car41 = x0[0] - math.cos(x0[2])*car_length/2 - math.sin(x0[2])*car_width/2
    car42 = x0[1] - math.sin(x0[2])*car_length/2 + math.cos(x0[2])*car_width/2

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
        xx4[i+1] = xx4[i] + T* ((1/m)*( (Cm1-Cm2*xx4[i])*guess[2*i] + m*xx5[i]*xx6[i] - df*math.sin(cf*math.atan(bf*(- math.atan((xx5[i] + lf*xx6[i])/xx4[i]) + guess[2*i+1])))*math.sin(guess[2*i+1]) - mu*xx4[i]**2 -0.6))
        xx5[i+1] = xx5[i] + T* ((1/m)*(-m*xx4[i]*xx6[i] + (df*math.sin(cf*math.atan(bf*(- math.atan((xx5[i] + lf*xx6[i])/xx4[i]) + guess[2*i+1])))*math.cos(guess[2*i+1]) + dr*math.sin(cr*math.atan(br*(- math.atan((xx5[i] - lr*xx6[i])/xx4[i]))))) ))
        xx6[i+1] = xx6[i] + T* ((1/Iz)*(lf*df*math.sin(cf*math.atan(bf*(- math.atan((xx5[i] + lf*xx6[i])/xx4[i]) + guess[2*i+1])))*math.cos(guess[2*i+1])-lr*dr*math.sin(cr*math.atan(br*(- math.atan((xx5[i] - lr*xx6[i])/xx4[i]))))))

    x0[0] = xx1[1]
    x0[1] = xx2[1]
    x0[2] = xx3[1]
    x0[3] = xx4[1]
    x0[4] = xx5[1]
    x0[5] = xx6[1]

    xx_hist.append(xx1[1])
    xy_hist.append(xx2[1])
    xtheta_hist.append(xx3[1])
    xvx_hist.append(xx4[1])

    ucl.append(u_cl1)
    ucl.append(u_cl2)

    row = [x0[0], x0[1], x0[2], x0[3], x0[4], x0[5],elapsed,u_cl1,u_cl2,now]
    writer.writerow(row)

    # updating data values
    line1.set_xdata(np.array([car11, car21, car41, car31, car11]))
    line1.set_ydata(np.array([car12, car22, car42, car32, car12]))
    line2.set_xdata(xx1)
    line2.set_ydata(xx2)
    line4.set_xdata(target_point[0])
    line4.set_ydata(target_point[1])
    #line5.set_xdata(proj_center_X)
    #line5.set_ydata(proj_center_Y)
    # drawing updated values
    figure.canvas.draw()
 
    # This will run the GUI event
    # loop until all UI events
    # currently waiting have been processed
    figure.canvas.flush_events()

    mpciter = mpciter+1


print('AVERAGE NL-MPC TIME: ', total_time/sim_steps)
print('max iteration time: ', max(elapsed_hist))
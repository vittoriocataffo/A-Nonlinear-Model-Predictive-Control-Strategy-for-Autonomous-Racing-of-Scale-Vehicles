# VITTORIO CATAFFO - 24/01/2022
# AUTONOMOUS RACING WITH OBSTACLES AVOIDANCE NMPC TARGET POINT APPROACH

import opengen as og
import casadi.casadi as cs 
import sys
import time
import numpy as np
import math

N = 50
T = 0.033

half_width = 2
car_width = 0.281;
car_length = 0.535;
lr = 0.160;
lf = 0.164;
m  = 4;
Iz  = 0.1217;
car_diag = math.sqrt(car_width**2+car_length**2);
security_distance = (half_width - car_diag/2)**2
r_obs = 1.5

duty_max = 1; 
duty_min = 0;

delta_max = math.pi/6; 
delta_min = -delta_max;

x  = cs.SX.sym('x'); 
y = cs.SX.sym('y');
theta = cs.SX.sym('theta');
vx = cs.SX.sym('vx');
vy = cs.SX.sym('vy');
omega = cs.SX.sym('omega');
states = [x, y, theta, vx, vy, omega];
n_states = len(states);

duty = cs.SX.sym('duty');
delta = cs.SX.sym('delta');
controls = [duty, delta];
n_controls = len(controls);

Cf = 0.0867;
Cr = 0.1632;
mu = 0.1;
Df = 42.5255;
Dr = 161.585;
Bf = 29.495;
Br = 26.9705;
Cm1 = 1.8073
Cm2 = -0.2511


f = cs.Function('f',[x, y, theta, vx, vy, omega, duty, delta],\
	[ vx*cs.cos(theta)-vy*cs.sin(theta),\
	vx*cs.sin(theta)+vy*cs.cos(theta),\
    omega,\
    (1/m)*( (Cm1-Cm2*vx)*duty + m*vy*omega - Df*cs.sin(Cf*cs.atan(Bf*(- cs.atan((vy + lf*omega)/vx) + delta)))*cs.sin(delta) - mu*vx**2 -0.6),\
    (1/m)*(-m*vx*omega + (Df*cs.sin(Cf*cs.atan(Bf*(- cs.atan((vy + lf*omega)/vx) + delta)))*cs.cos(delta) + Dr*cs.sin(Cr*cs.atan(Br*(- cs.atan((vy - lr*omega)/vx))))) ),\
    (1/Iz)*(lf*Df*cs.sin(Cf*cs.atan(Bf*(- cs.atan((vy + lf*omega)/vx) + delta)))*cs.cos(delta)-lr*Dr*cs.sin(Cr*cs.atan(Br*(- cs.atan((vy - lr*omega)/vx))))) ]);

U = cs.SX.sym('U',n_controls*N);
P = cs.SX.sym('P',n_states + n_controls + 2 + 2*N + 4);
X = cs.SX.sym('X',n_states*(N+1))

X[0] = P[0]
X[1] = P[1]
X[2] = P[2]
X[3] = P[3]
X[4] = P[4]
X[5] = P[5]

obj = 0
f1 = []
c = 0
preU1 = P[6]
preU2 = P[7]

for i in range(N):
	st1 = X[6*(i)]
	st2 = X[6*(i)+1]
	st3 = X[6*(i)+2]
	st4 = X[6*(i)+3]
	st5 = X[6*(i)+4]
	st6 = X[6*(i)+5]
	con1 = U[2*(i)]
	con2 = U[2*(i)+1]
	obj = obj + 10*(con1-preU1)**2 + 10*(con2-preU2)**2 
	f_value = f(st1, st2, st3, st4, st5, st6, con1, con2)
	st_next1 = st1 + T*f_value[0]
	st_next2 = st2 + T*f_value[1]
	st_next3 = st3 + T*f_value[2]
	st_next4 = st4 + T*f_value[3]
	st_next5 = st5 + T*f_value[4]
	st_next6 = st6 + T*f_value[5]
	X[6*(i+1)] = st_next1
	X[6*(i+1)+1] = st_next2
	X[6*(i+1)+2] = st_next3
	X[6*(i+1)+3] = st_next4
	X[6*(i+1)+4] = st_next5
	X[6*(i+1)+5] = st_next6
	f1 = cs.vertcat(f1, (st1 - P[2*(i+1)+8])**2 + (st2 - P[2*(i+1)+9])**2 )
	f1 = cs.vertcat(f1, con1-preU1, con2-preU2, st_next4) 
	preU1 = con1
	preU2 = con2
	c += cs.fmax(0, r_obs**2 - (st_next1-P[n_states + n_controls + 2 + 2*N])**2 - (st_next2-P[n_states + n_controls + 2 + 2*N + 1])**2)
	c += cs.fmax(0, r_obs**2 - (st_next1-P[n_states + n_controls + 2 + 2*N + 2])**2 - (st_next2-P[n_states + n_controls + 2 + 2*N + 3])**2)
	obj = obj + ( ((-cs.atan((st5+lf*st6)/st4)+con2)) - (-cs.atan((st5-lr*st6)/st4)) )**2 
	

obj = obj + 10*(st_next1-P[8])**2 + 10*(st_next2-P[9])**2


umin = [duty_min, delta_min]*N
umax = [duty_max, delta_max]*N
bounds = og.constraints.Rectangle(umin, umax)

deltau1 = 0.5
deltau2 = 0.025 #0.025
bmin = [0, -deltau1, -deltau2, 1]*N
bmax = [security_distance, deltau1, deltau2, 5]*N
set_c = og.constraints.Rectangle(bmin,bmax)

problem = og.builder.Problem(U, P, obj).with_aug_lagrangian_constraints(f1, set_c).with_penalty_constraints(c).with_constraints(bounds)	
build_config = og.config.BuildConfiguration()\
    .with_build_directory("dynamic_my_optimizer")\
    .with_build_mode("release")\
    .with_build_python_bindings()
meta = og.config.OptimizerMeta()\
    .with_optimizer_name("dynamic_racing_target_point")
solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-4)\
    .with_initial_tolerance(1e-4)\
    .with_max_outer_iterations(1)\
    .with_delta_tolerance(1e-1)\
    .with_penalty_weight_update_factor(10.0)\
    .with_max_duration_micros(30000)
builder = og.builder.OpEnOptimizerBuilder(problem, 
                                          meta,
                                          build_config, 
                                          solver_config) \
    .with_verbosity_level(1)
builder.build()


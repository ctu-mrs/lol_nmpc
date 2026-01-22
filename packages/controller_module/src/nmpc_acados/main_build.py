
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from quadrotor_model import export_quadrotor_ode_model
import numpy as np
import scipy.linalg
from utils import plot_quadrotor, yaml_parse
import yaml
import matplotlib.pyplot as plt
import time
import os
from math import sqrt

# Find the location of the workin directory
dir_path = os.path.dirname(os.path.realpath(__file__)) # load location path

#import settings from yaml:
try:
    with open(f'{dir_path}/base_config.yaml') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
except:
    data = {}
# create ocp object to formulate the OCP
ocp = AcadosOcp()

# set model
model = export_quadrotor_ode_model(data)
ocp.model = model

Tf = yaml_parse(data, "ac_horizon", 1)
nx = model.x.size()[0]

nu = model.u.size()[0]
ny = nx + nu
ny_e = nx
# print(f"nx:{nx}, nu: {nu}") # = 13, 4
N = yaml_parse(data, "ac_num_iter", 20)

K=1.0
# kd_xy= 0.000 * K
# ki_xy= 0.02 * K
# kp_xy= 0.06 * K
kd_xy=0.00
# ki_xy=0.002 * K 
# kp_xy=0.03 * K
ki_xy=0.0044
kp_xy=0.066

# print()
kd_z= 0.0
ki_z= 0.03
kp_z= 0.14

# kd_xy: 0.0015
# ki_xy: 0.08
# kp_xy: 0.042

max_integral_xy = 0.3
max_integral_z = 0.3
# max_integral_xy = 1.0
# max_integral_z = 1.0

# max_integral_PID = 1.5

# kd = 0.003
# ki = 0.10
# kp = 0.3
cdrag = yaml_parse(data, "drag", [0.0, 0.0, 0.0])
# tau_mot = yaml_parse(data,"tau_mot",0.040)
tau_mot = 0.03
# set dimensions
ocp.dims.N = N
q_ref_init = np.array([1.0, 0.0, 0.0, 0.0])
inertia = yaml_parse(data, "inertia", [0.001, 0.001, 0.0014])
actuator_max_thrust = yaml_parse(data, "thrust_actuator_max", 17)
mot0_pos = yaml_parse(data, "mot0_pos", [  0.15, -0.15])
mot1_pos = yaml_parse(data, "mot1_pos", [ -0.15,  0.15])
mot2_pos = yaml_parse(data, "mot2_pos", [ -0.15, -0.15])
mot3_pos = yaml_parse(data, "mot3_pos", [  0.15,  0.15])
ocp.parameter_values = np.array([yaml_parse(data, "mass", 0.85), mot0_pos[0], mot0_pos[1], mot1_pos[0], mot1_pos[1], mot2_pos[0], mot2_pos[1], mot3_pos[0], mot3_pos[1], inertia[0], inertia[1], inertia[2], yaml_parse(data, "ctau", 0.05), cdrag[0],cdrag[1],cdrag[2],q_ref_init[0],q_ref_init[1],q_ref_init[2],q_ref_init[3],kd_xy,kd_xy,kd_z,kp_xy,kp_xy,kp_z,ki_xy,ki_xy,ki_z,tau_mot,actuator_max_thrust])


# Constraints
omega_max = np.array(yaml_parse(data, "max_rot_speed", [10.0, 10.0, 4.0]))  # [rad/s]
throttle_collective_max = 1.0 #yaml_parse(data, "thrust_collective_max", 850)       # [N] per motor
throttle_collective_min = 0.0 #yaml_parse(data, "thrust_collective_min", 0.0)       # [N]
throttle_actuator_max = 1.0 #yaml_parse(data, "thrust_actuator_max", 40.0)  # [N]
throttle_actuator_min = 0.0 #yaml_parse(data, "thrust_actuator_min", 1.5)  # [N]


# set constraints
Jbx = np.zeros((6, nx)) # matrix for assigning constrain matrix "omega_max" to corresponding state variables
#max omega
Jbx[0, 10] = 1.0
Jbx[1, 11] = 1.0
Jbx[2, 12] = 1.0

#max z
Jbx[3, 13] = 1.0
Jbx[4, 14] = 1.0
Jbx[5, 15] = 1.0

#max T
# Jbx[6, 16] = 1.0
# Jbx[7, 17] = 1.0
# Jbx[8, 18] = 1.0
# Jbx[9, 19] = 1.0


ocp.constraints.Jbx = Jbx
print("omega_max",omega_max)
print("integra",np.array([max_integral_xy,max_integral_xy,max_integral_z]))
ocp.constraints.lbx = np.array([-omega_max[0],-omega_max[1],-omega_max[2], -max_integral_xy,-max_integral_xy,-max_integral_z])
ocp.constraints.ubx = np.array([omega_max[0],omega_max[1],omega_max[2],max_integral_xy,max_integral_xy,max_integral_z])
print("ocp.constraints.lbx",ocp.constraints.lbx)
print("ocp.constraints.ubx",ocp.constraints.ubx)

Jbu = np.identity(nu) # matrix for assigning thrust constrain matrix to corresponding input variables
# ocp.constraints.Jbu = Jbu
# ocp.constraints.lbu = thrust_min * np.ones((nu,))
# ocp.constraints.ubu = thrust_max * np.ones((nu,))
ocp.constraints.Jbu = Jbu
ocp.constraints.lbu = np.concatenate((np.array([throttle_collective_min]),-1*omega_max))
ocp.constraints.ubu = np.concatenate((np.array([throttle_collective_max]),omega_max))

mat_m = np.zeros((4, 4))
mat_m[0,:] = 1.0
Alloc =  np.zeros((4, 4))
Alloc[:,0] = 1.0
Alloc[0,1] = -0.7071
Alloc[1,1] = 0.7071
Alloc[2,1] = -0.7071
Alloc[3,1] = 0.7071
Alloc[0,2] = -0.7071
Alloc[1,2] = 0.7071
Alloc[2,2] = 0.7071
Alloc[3,2] = -0.7071
Alloc[0,3] = -1
Alloc[1,3] = -1
Alloc[2,3] = 1
Alloc[3,3] = 1
# px4_pid:
#   thrust_scale: [1.0, 1.0, 1.0, 1.0]
#   roll_scale: [-0.7071, 0.7071, -0.7071, 0.7071]
#   pitch_scale: [-0.7071, 0.7071, 0.7071, -0.7071]
#   yaw_scale: [-1.0, -1.0, 1.0, 1.0]
temp_d = np.identity(nu)
temp_d[1,1] = kp_xy
temp_d[2,2] = kp_xy
temp_d[3,3] = kp_z
temp_d = np.matmul(Alloc,temp_d)
ocp.constraints.D = temp_d
temp_c = np.zeros((4, nx))
bef_c = np.zeros((4, 4))
bef_c[:,0] = 0.0
bef_c[:,1] =  kp_xy * Alloc[:,1]
bef_c[:,2] =  kp_xy * Alloc[:,2]
bef_c[:,3] =  kp_z  * Alloc[:,3]
temp_c[:,9:13] = - bef_c
ocp.constraints.C = temp_c
ocp.constraints.ug = np.ones(4) * throttle_actuator_max
ocp.constraints.lg = np.ones(4) * throttle_actuator_min


# set cost
Q_p_xy = yaml_parse(data, "Q_position_xy", 200)
Q_p_z = yaml_parse(data, "Q_position_z", 500)
Q_r_xy = yaml_parse(data, "Q_rotation_xy", 10)
Q_r_z = yaml_parse(data, "Q_rotation_z", 10)
Q_v = yaml_parse(data, "Q_velocity", 1)
Q_o = yaml_parse(data, "Q_omega", 1)
Q_int = yaml_parse(data, "Q_int", 0.1)
Q_T = yaml_parse(data, "Q_thrust", 0.01)
R_col_thrust = yaml_parse(data, "R_collective_thrust", 0.1)
R_ome_in = yaml_parse(data, "R_omega_in", 0.1)
Q_mat = np.diag([Q_p_xy, Q_p_xy, Q_p_z, Q_r_xy, Q_r_xy, Q_r_z,
                 Q_v, Q_v, Q_v, Q_o, Q_o, Q_o, Q_int,Q_int,Q_int,Q_T,Q_T,Q_T,Q_T])
R_mat = np.diag([R_col_thrust, R_ome_in, R_ome_in, R_ome_in])

ny = nx + nu
ny_e = nx

# set cost
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.cost.cost_type_e = 'NONLINEAR_LS'
W_mat = scipy.linalg.block_diag(Q_mat, R_mat)
ocp.cost.W = W_mat
ocp.cost.W_e = Q_mat

#this is not important as it will be overwriten in cpp code
mass_value = yaml_parse(data, "mass", 0.85)
# initial references
# hover_prop = mass_value * 9.8066
hover_prop = sqrt(((mass_value/4.0) * 9.8066)/17.0)
# px py pz
# qx qy qz
# vx vy vz
# omegax omegay omegaz
# PID integrator x y z
# motor thrusts
ocp.cost.yref = np.array([0, 2, 0, \
                          0, 0, 0, \
                          0, 0, 0, \
                          0, 0, 0, \
                          0, 0, 0, \
                          hover_prop, hover_prop, hover_prop,hover_prop, \
                          hover_prop, 0, 0, 0]) # Control inputs
# px py pz
# qx qy qz
# vx vy vz
# omegax omegay omegaz
# PID integrator x y z
 # motor thrusts
ocp.cost.yref_e = np.array([0, 2, 0,\
                            0, 0, 0,\
                            0, 0, 0,\
                            0, 0, 0,\
                            0, 0, 0,\
                            hover_prop, hover_prop, hover_prop,hover_prop])


init_state = np.zeros([nx])
init_state[3] = 1 # quaternion must be unitary
init_state[16] = hover_prop # hover thust
init_state[17] = hover_prop # hover thust
init_state[18] = hover_prop # hover thust
init_state[19] = hover_prop # hover thust
ocp.constraints.x0 = init_state


# set options
ocp.solver_options.tf = Tf # set prediction horizon
ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "PARTIAL_CONDENSING_HPIPM", "FULL_CONDENSING_HPIPM"
ocp.solver_options.nlp_solver_type = "SQP_RTI"     # "SQP", "SQP_RTI"
ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # "GAUSS_NEWTON", "EXACT"
ocp.solver_options.integrator_type = "ERK"   # "ERK", "IRK", "GNSF"
ocp.solver_options.hpipm_mode = "SPEED_ABS"

# add compilation flags
# ocp.solver_options.ext_fun_compile_flags = "-march=native -O3"

# ocp.solver_options.levenberg_marquardt = 0.0
#ocp.solver_options.sim_method_jac_reuse = True
#ocp.solver_options.sim_method_newton_iter = 1
#ocp.solver_options.sim_method_num_stages = 3
#ocp.solver_options.sim_method_num_steps = 1
# ocp.solver_options.print_level = 1
new_time_steps = np.zeros(N)
new_time_steps[0] = 0.02
new_time_steps[1] = 0.02
new_time_steps[2] = 0.02
rest_dt = (Tf-0.02)/float(N-2) 
sumtest = 0.06          
for i in range(3,N-1):
    new_time_steps[i] = rest_dt
    sumtest += rest_dt
print(sumtest)
ocp.solver_options.time_steps=new_time_steps

ocp.code_export_directory = dir_path + "/c_generated_code"

# this will generate c code
ocp_solver = AcadosOcpSolver(ocp, json_file = f'{dir_path}/acados_ocp.json')
# Tf

acados_integrator = AcadosSimSolver(ocp, json_file = f'{dir_path}/acados_ocp.json' )
dt_sim = ocp_solver.acados_ocp.solver_options.Tsim
# this will do simulation also in python
 
# status = ocp_solver.solve()
# ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")
# 
# if status != 0:
    # raise Exception(f'acados returned status {status}.')

print("lbu",ocp.constraints.lbu)
print("ubu",ocp.constraints.ubu)
# 
# # get solution
Nsim = 40
# Nsim = 600
simX = np.ndarray((Nsim+1, nx))
simU = np.ndarray((Nsim, nu))
solvetime = np.zeros((Nsim+1, 1))

xcurrent = init_state
simX[0,:]=xcurrent
yref = ocp.cost.yref
yref_N = ocp.cost.yref_e
for i in range(Nsim):
    # set initial state constraint
    ocp_solver.set(0, "lbx", xcurrent)
    ocp_solver.set(0, "ubx", xcurrent)

    # update yref
    for j in range(N):
        ocp_solver.set(j, "yref", yref)
    ocp_solver.set(N, "yref", yref_N)
    
    # solve ocp
    bef = int(round(time.time() * 1000000))
    status = 0
    # status = ocp_solver.solve()
    stime = int(round(time.time() * 1000000))-bef
    print(stime,'us')
    solvetime[i,0]=stime
    if status not in [0, 2]:
        ocp_solver.print_statistics()
        # plot_robot(
        #         np.linspace(0, T_horizon / N_horizon * i, i + 1),
        #         F_max,
        #         simU[:i, :],
        #         simX[: i + 1, :],
        # )
        raise Exception(f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}")

    if status == 2:
        print(f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}")


    # print('size',ocp_solver.get(0, "u").size)
    # simU[i, :] = ocp_solver.get(0, "u")
    # print(simU[i, :])
    # simU[i, :] = np.array([hover_prop,0,0,2])
    simU[i, :] = np.array([hover_prop,3,0,0])
    
    # print("xcurrent omega",xcurrent[10:13])
    # print("xcurrent z",xcurrent[13:16])
    # print("xcurrent T",xcurrent[16:20])
    # simulate system
    acados_integrator.set("x", xcurrent)
    acados_integrator.set("u", simU[i, :])
    
    status = acados_integrator.solve()
    if status != 0:
        raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")
    
    # update state
    xcurrent = acados_integrator.get("x")
    simX[i + 1, :] = xcurrent

    print("u",simU[i,0])
    print(xcurrent[2])
    # print("x",simX[:,:])
    # for i in range(N):
    #     simX[i,:] = ocp_solver.get(i, "x")
    #     simU[i,:] = ocp_solver.get(i, "u")
    # simX[N,:] = ocp_solver.get(N, "x")

# 
# plot_quadrotor(np.linspace(0, Tf, N+1), thrust_max, simU, simX, latexify=False)
plot_num = 710
plt.figure(1)
plt.subplot(plot_num+1)
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 0],label='x')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 1],label='y')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 2],label='z')
plt.ylabel('pos')
plt.xlabel('time [s]')
plt.legend()
plt.subplot(plot_num+2)
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 7],label='x')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 8],label='y')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 9],label='z')
plt.ylabel('vel')
plt.xlabel('time [s]')
plt.legend()
plt.subplot(plot_num+3)
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 10],label='x')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 11],label='y')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 12],label='z')
plt.axhline(y = np.max(simX[:, 10]), color = 'r', linestyle = '--',label=str(np.max(simX[:, 10])))
plt.axhline(y = 3.0, color = 'g', linestyle = '--')
# plt.axhline(y = 2.0, color = 'g', linestyle = '--')
plt.grid()
# plt.axhline(y = np.max(simX[:, 11] ), color = 'b', linestyle = '--',label=str(np.max(simX[:, 11])))
plt.ylabel('omega')
plt.xlabel('time [s]')
plt.legend()
plt.subplot(plot_num+4)
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 13],label='zx')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 14],label='zy')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 15],label='zz')
plt.ylabel('PID integrator')
plt.xlabel('time [s]')
plt.legend()
plt.subplot(plot_num+5)
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 16],label='T1')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 17],label='T2')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 18],label='T1')
plt.plot(list(dt_sim*range(Nsim+1)),simX[:, 19],label='T4')
plt.ylabel('motor throttle')
plt.xlabel('time [s]')
plt.legend()
plt.subplot(plot_num+6)
# plt.plot(list(range(1,Nsim+1)),simU[:, 0],label='t')
plt.plot(list(dt_sim*range(1,Nsim+1)),simU[:, 1],label='wcx')
plt.plot(list(dt_sim*range(1,Nsim+1)),simU[:, 2],label='wcy')
plt.plot(list(dt_sim*range(1,Nsim+1)),simU[:, 3],label='wcz')
plt.ylabel('command')
plt.xlabel('time [s]')
plt.legend()
plt.subplot(plot_num+7)
plt.plot(list(dt_sim*range(Nsim+1)), actuator_max_thrust * np.multiply(simX[:, 16] , simX[:, 16]), label='T1')
plt.plot(list(dt_sim*range(Nsim+1)), actuator_max_thrust * np.multiply(simX[:, 17] , simX[:, 17]), label='T1')
plt.plot(list(dt_sim*range(Nsim+1)), actuator_max_thrust * np.multiply(simX[:, 18] , simX[:, 18]), label='T1')
plt.plot(list(dt_sim*range(Nsim+1)), actuator_max_thrust * np.multiply(simX[:, 19] , simX[:, 19]), label='T1')
plt.ylabel('motor thrusts')
plt.xlabel('time [s]')
plt.legend()
# plt.subplot(plot_num+7)
# plt.plot(list(range(Nsim+1)),solvetime[:, 0])
# plt.ylabel('solvetime')
# plt.xlabel('time [s]')
plt.show()

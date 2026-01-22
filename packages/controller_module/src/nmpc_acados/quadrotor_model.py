#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl

# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

from acados_template import AcadosModel
from casadi import DM, MX, SX, vertcat, sin, cos, Function, inv, cross, mtimes, diag, sqrt, norm_2
from quaternion import rotate_quat, quat_mult, quat_error
import numpy as np
from utils import yaml_parse

def export_quadrotor_ode_model(data : dict) -> AcadosModel:

    model_name = 'quadrotor_ode'

    # constants
    m = SX.sym('m')                # mass in [kg]
    # l = SX.sym('l')                # arm length
    mot0_pos = SX.sym('mot0_pos', 2)
    mot1_pos = SX.sym('mot1_pos', 2)
    mot2_pos = SX.sym('mot2_pos', 2)
    mot3_pos = SX.sym('mot3_pos', 2)
    I_diag = SX.sym('I_diag', 3)   # Inertia
    I = diag(I_diag)
    I_inv = diag(1/I_diag)         # Inertia inverse
    ctau = SX.sym('ctau')          # thrust torque coeff.
    cdrag = SX.sym('cdrag',3)        # linear air drag coeff.
    q_ref = SX.sym("quat_ref",4)
    kd = SX.sym('kd', 3)
    kp = SX.sym('kp', 3)
    ki = SX.sym('ki', 3)
    tau_mot = SX.sym('tau_mot') #motor tau
    a_max_thr = SX.sym('a_max_thr') #actuator_max_thrust
    eye =  diag([1.0,1.0,1.0])

    # set up states & controls
    p = SX.sym('p', 3)
    q = SX.sym('q', 4)
    v = SX.sym('v', 3)
    w = SX.sym('w', 3)
    z = SX.sym('z', 3)
    T = SX.sym('T', 4)
    thr = SX.sym('thr', 4)
    wc = SX.sym('wc', 3)
    # T = SX.sym('thrust', 4)
    t = SX.sym('t', 1)

    x = vertcat(p, q, v, w, z,thr)
    u = vertcat(t, wc)

    g_ = yaml_parse(data, "g", 9.806)
    g = SX([0, 0, -g_])
    print("g",g)

    # xdot
    p_dot = SX.sym('p_dot', 3)
    q_dot = SX.sym('q_dot', 4)
    v_dot = SX.sym('v_dot', 3)
    w_dot = SX.sym('w_dot', 3)
    z_dot = SX.sym('z_dot', 3)
    # T_dot = SX.sym('T_dot', 4)
    thr_dot = SX.sym('thr_dot', 4)

    xdot = vertcat(p_dot, q_dot, v_dot, w_dot,z_dot,thr_dot)

    # algebraic variables
    # z = None

    # parameters
    par = vertcat(m, mot0_pos, mot1_pos, mot2_pos, mot3_pos, I_diag,
                  ctau, cdrag, q_ref, kd, kp, ki, tau_mot, a_max_thr)

    #                   ^
    #        *3   *0    |
    #          \ /      |
    #           .       |
    #          / \      |x
    #        *1   *2    |
    #                   |
    #     <_____________|
    #            y

    # mot0_pos = yaml_parse(data, "mot0_pos", [  0.15, -0.15])
    # mot1_pos = yaml_parse(data, "mot1_pos", [ -0.15,  0.15])
    # mot2_pos = yaml_parse(data, "mot2_pos", [ -0.15, -0.15])
    # mot3_pos = yaml_parse(data, "mot3_pos", [  0.15,  0.15])

    
    T[0] = a_max_thr * thr[0] * thr[0]
    T[1] = a_max_thr * thr[1] * thr[1]
    T[2] = a_max_thr * thr[2] * thr[2]
    T[3] = a_max_thr * thr[3] * thr[3]
    #omega dot equation
    omega_dot_equation = mtimes(I_inv, vertcat(
        (+ T[0]*mot0_pos[1] + T[1]*mot1_pos[1]
         + T[2]*mot2_pos[1] + T[3]*mot3_pos[1]),
        (-T[0]*mot0_pos[0] - T[1]*mot1_pos[0] +
         -T[2]*mot2_pos[0] - T[3]*mot3_pos[0]),
        ctau*(-T[0]-T[1]+T[2]+T[3])) - cross(w, mtimes(I, w)))
    
    #pid equation
    tauc = kp*(wc-w) + ki*z #+ kd * omega_dot_equation
    
    #commanded thrust
    #   thrust_scale: [1.0, 1.0, 1.0, 1.0]
    #   roll_scale: [-0.7071, 0.7071, -0.7071, 0.7071]
    #   pitch_scale: [-0.7071, 0.7071, 0.7071, -0.7071]
    #   yaw_scale: [-1.0, -1.0, 1.0, 1.0]
    # thr_c = mtimes(inv(M),vertcat(t, tauc))
    thr_c = SX.sym('thr_c', 4)
    # print("inv M")
    # print(inv(M))
    # outputs(i) = roll * roll_scale_[i] + pitch * pitch_scale_[i] + thrust * thrust_scale_[i];
    thr_c[0] = t - 0.7071 * tauc[0] - 0.7071 * tauc[1] - 1 * tauc[2]
    thr_c[1] = t + 0.7071 * tauc[0] + 0.7071 * tauc[1] - 1 * tauc[2]
    thr_c[2] = t - 0.7071 * tauc[0] + 0.7071 * tauc[1] + 1 * tauc[2]
    thr_c[3] = t + 0.7071 * tauc[0] - 0.7071 * tauc[1] + 1 * tauc[2]

    # drag portion
    q_normalized = q/norm_2(q)
    q_conj = vertcat(q_normalized[0], -q_normalized[1], -q_normalized[2], -q_normalized[3])
    # q_conj = q_conj/norm_1(q_conj)
    print(q_conj)
    v_body = rotate_quat(q_conj, v)
    print(v_body)
    f_drag_body = (cdrag/m) * v_body
    print(f_drag_body)
    # drag coefs map from vel to accel directly so mass is not needed
    a_drag = rotate_quat(q_normalized, f_drag_body)

    #dynamics
    f_expl = vertcat(
        v,
        0.5*quat_mult(q_normalized, vertcat(0, w)),
        rotate_quat(q_normalized, vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/m)) + g - a_drag,
        omega_dot_equation,
        wc-w,
        -(1/tau_mot) * thr + (1/tau_mot) * thr_c 
    )

    f_impl = xdot - f_expl

    model = AcadosModel()
    print(type(f_expl))
    print(isinstance(f_expl, SX))
    # model.f_impl = f_impl
    # model.f_expl = f_expl
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    q_att = quat_error(q_normalized,q_ref)
    model.cost_y_expr = vertcat(p, q_att, v, w, z, T, u[0]*u[0]*a_max_thr*4,u[1],u[2],u[3])
    model.cost_y_expr_e = vertcat(p, q_att, v, w, z ,T)
    model.x = x
    model.xdot = xdot
    model.u = u
    # model.z = z
    model.p = par
    model.name = model_name
    # model.Tc = T_c

    return model

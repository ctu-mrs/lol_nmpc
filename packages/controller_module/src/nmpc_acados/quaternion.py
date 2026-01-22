from casadi import vertcat,sqrt
import numpy as np
import math

# For casadi
# Quaternion Multiplication
def quat_mult(q1,q2):
    ans = vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
           q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
           q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
           q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

# Quaternion-Vector Rotation
def rotate_quat(q1,v1):
    ans = quat_mult(quat_mult(q1, vertcat(0, v1)), vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec

# For Numpy
def skew(v):
  return np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])

def conj(q):
  return np.array([q[0], -q[1], -q[2], -q[3]], ndmin=2).T

def Ql(q):
  if q.ndim == 1: q = np.array(q, ndmin=2).T
  QL = np.zeros((4,4))
  QL[0, 1:4] = -q[1:4].flatten()
  QL[1:4, 0] = q[1:4].flatten()
  QL[1:4, 1:4] = skew(q[1:4])
  return q[0] * np.eye(4) + QL

def Qr(q):
  if q.ndim == 1: q = np.array(q, ndmin=2).T
  QR = np.zeros((4,4))
  QR[0, 1:4] = -q[1:4].flatten()
  QR[1:4, 0] = q[1:4].flatten()
  QR[1:4, 1:4] = skew(q[1:4]).T
  return q[0] * np.eye(4) + QR

def qtimes(q1, q2):
  return np.matmul(Ql(q1), q2)

def qRot(q):
  R = np.matmul(Ql(q), Qr(conj(q)))
  return R[1:4, 1:4]

def qRotv(q, v):
  return np.matmul(qRot(q), v)

def angleAxisToQuaternion(angle = None, axis = None):
  assert axis is not None
  anorm = np.linalg.norm(axis)
  if angle is None:
    angle = anorm
  ca2 = np.cos(angle/2.0)
  sa2 = np.sin(angle/2.0)
  dir = axis/anorm
  return np.array([ca2, sa2 * dir[0], sa2 * dir[1], sa2 * dir[2]], ndmin=2)

def eulerToQuaternion(angle):
  qz = np.array([np.cos(angle[2]/2), 0, 0, np.sin(angle[2]/2)], ndmin=2).T
  qy = np.array([np.cos(angle[1]/2), 0, np.sin(angle[1]/2), 0], ndmin=2).T
  qx = np.array([np.cos(angle[0]/2), np.sin(angle[0]/2), 0, 0], ndmin=2).T
  return qtimes(qz, qtimes(qy, qx))

def quat_error(q, q_ref):
    q_aux = vertcat(q[0, :] *
                    q_ref[0, :] +
                    q[1, :] *
                    q_ref[1, :] +
                    q[2, :] *
                    q_ref[2, :] +
                    q[3, :] *
                    q_ref[3, :], -
                    q[1, :] *
                    q_ref[0, :] +
                    q[0, :] *
                    q_ref[1, :] +
                    q[3, :] *
                    q_ref[2, :] -
                    q[2, :] *
                    q_ref[3, :], -
                    q[2, :] *
                    q_ref[0, :] -
                    q[3, :] *
                    q_ref[1, :] +
                    q[0, :] *
                    q_ref[2, :] +
                    q[1, :] *
                    q_ref[3, :], -
                    q[3, :] *
                    q_ref[0, :] +
                    q[2, :] *
                    q_ref[1, :] -
                    q[1, :] *
                    q_ref[2, :] +
                    q[0, :] *
                    q_ref[3, :])
    # attitude errors. SQRT have small quantities added (1e-3) to alleviate the derivative
    # not being defined at zero, and also because it's in the denominator
    q_att_denom = sqrt(q_aux[0] * q_aux[0] + q_aux[3] * q_aux[3] + 1e-3)
    q_att = (
        vertcat(
            q_aux[0] *
            q_aux[1] -
            q_aux[2] *
            q_aux[3],
            q_aux[0] *
            q_aux[2] +
            q_aux[1] *
            q_aux[3],
            q_aux[3],
        ) /
        q_att_denom)
    return q_att

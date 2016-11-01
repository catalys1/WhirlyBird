import numpy as np
import params as P
from WhirlybirdController import WhirlybirdController


class WhirlybirdDynamics(object):

  def __init__(self, controller=None):

    if controller:
      self.controller = controller
    else:
      self.controller = WhirlybirdController()

    self.state = np.matrix([
      [P.phi0],
      [P.theta0],
      [P.psi0],
      [P.phidot0],
      [P.thetadot0],
      [P.psidot0]
    ])


  def propogateDynamics(self, u):
    'u: reference inputs (for now just h_ref)'
    k1 = self.Derivatives(self.state, u)
    k2 = self.Derivatives(self.state + P.Ts/2*k1, u)
    k3 = self.Derivatives(self.state + P.Ts/2*k2, u)
    k4 = self.Derivatives(self.state + P.Ts*k3, u)
    self.state += P.Ts/6*(k1 + 2*k2 + 2*k3 + k4)


  def Derivatives(self, state, u):
    # unpack state and inputs
    phi   = state.item(0)
    theta = state.item(1)
    psi   = state.item(2)
    phidot= state.item(3)
    thetadot= state.item(4)
    psidot = state.item(5)

    F, tau = self.controller.getForces(u, state)

    fr = (P.d*F-tau) / (2.0*P.d)
    fl = F - fr

    # get physical parameters from the params file
    g, Jx, Jy, Jz, km = P.g, P.Jx, P.Jy, P.Jz, P.km
    l1, l2, m1, m2, d, h, r = P.l1, P.l2, P.m1, P.m2, P.d, P.h, P.r

    # pre-compute sine and cosine values for these states
    s_phi = np.sin(phi)
    s_theta = np.sin(theta)
    c_phi = np.cos(phi)
    c_theta = np.cos(theta)
    c_psi = np.cos(psi)

    M = np.matrix([
      [Jx, 0, -Jx*s_theta],
      [0, m1*l1**2 + m2*l2**2 + Jy*c_phi**2 + Jz*s_phi**2, (Jy-Jz)*s_phi*c_phi*c_theta],
      [-Jx*s_phi, (Jy-Jz)*s_phi*c_phi*c_theta, 
        (m1*l1**2+m2*l2**2+Jy*s_phi**2)*c_theta**2+Jx*s_theta**2]
    ])

    Q = np.matrix([
      [d*(fl - fr)],
		  [l1*(fl + fr)*c_phi],
		  [l1*(fl + fr)*c_theta*s_phi + d*(fr - fl)*s_theta]
    ])

    dP_dq = np.matrix([
      [0],
			[(m1*l1-m2*l2)*g*c_theta],
			[0]
    ])

    c = np.matrix([
      [-thetadot**2*(Jz - Jy)*s_phi*c_phi + psidot**2*(Jz - Jy)*s_phi*c_phi*c_theta**2 - phidot*psidot*c_theta*(Jx - (Jz - Jy)*(c_phi**2 - s_phi**2))],
      [psidot**2*s_theta*c_theta*(-Jx + m1*l1**2 + m2*l2**2 + Jy*s_phi**2 + Jz*c_phi**2) - 2*phidot*thetadot*(Jz - Jy)*s_phi*c_phi - phidot*psidot*c_theta*(-Jx + (Jz - Jy)*(c_phi**2 - s_phi**2))],
      [thetadot**2*(Jz - Jy)*s_phi*c_phi*s_theta - phidot*thetadot*c_theta*(Jx + (Jz - Jy)*(c_phi**2 - s_phi**2)) - 2*phidot*psidot*(Jz - Jy)*c_theta**2*s_phi*c_phi + 2*thetadot*psidot*s_theta*c_theta*(Jx - m1*l1**2 - m2*l2**2 - Jy*s_phi**2 - Jz*c_phi**2)]])

    qdot = np.linalg.inv(M) * (Q-c-dP_dq)

    return np.matrix([
      [phidot],
      [thetadot],
      [psidot],
      [qdot.item(0)],
      [qdot.item(1)],
      [qdot.item(2)]
    ])


  def Outputs(self):
    return self.state[:3].T.tolist()[0]


  def States(self):
    return self.state.T.tolist()[0]

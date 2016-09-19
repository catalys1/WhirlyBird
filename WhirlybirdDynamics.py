import numpy as np
import params as P


class WhirlybirdDynamics(object):

  def __init__(self):

    self.state = np.matrix([
      [phi],
      [theta],
      [psi],
      [phidot],
      [thetadot],
      [psidot]
    ])


  def propogateDynamics(self, u):
    k1 = self.Derivatives(self.state, u)
    k2 = self.Derivatives(self.state + P.Ts/2*k1, u)
    k3 = self.Derivatives(self.state + P.Ts/2*k2, u)
    k4 = self.Derivatives(self.state + P.Ts*k3, u)
    self.state += P.Ts/6*(k1 + 2*k2 + 2*k3 + k4)


  def Derivatives(self, state, u):
    pass


  def Outputs(self):
    return self.state[:3].T.tolist()[0]


  def States(self):
    return self.state.T.tolist()[0]

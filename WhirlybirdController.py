import numpy as np
import params as P


class WhirlybirdController(object):

  CONTROLLER_PD = 1  

  def __init__(self, ctrl_type = CONTROLLER_PD):
    self.ctrl_type = ctrl_type


  def getForces(self, ref_input, states):
    if self.ctrl_type == WhirlybirdController.CONTROLLER_PD:
      return self.getPDForces(ref_input, states)


  def getPDForces(self, ref_input, states):
    th, th_dot = states
    th_r = ref_input
    
    Feq = (P.m1*P.l1 - P.m2*P.l2)*P.g*np.cos(th) / P.l1
    #Feq = (P.m1*P.l1 - P.m2*P.l2)*P.g / P.l1
    Ftilde = P.kp_th*(th_r - th) - P.kd_th*th_dot
    # Ftilde = F - Feq
    F = Ftilde + Feq

    return [F]


  def changeCtrlType(self, ctrl_type):
    self.ctrl_type = ctrl_type

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
		phi = states.item(0)
		th = states.item(1)
		psi = states.item(2)
		phi_dot = states.item(3)
		th_dot = states.item(4)
		psi_dot = states.item(5)
		th_r = ref_input[0]
		psi_r = ref_input[1]
		
		# Longitudinal
		Feq = (P.m1*P.l1 - P.m2*P.l2)*P.g*np.cos(th) / P.l1
		#Feq = P.Feq * np.cos(th)
		Ftilde = P.kp_th*(th_r - th) - P.kd_th*th_dot
		F = Ftilde + Feq

		# Lateral
		phi_r = P.kp_psi*(psi_r - psi) - P.kd_psi*psi_dot
		tau = P.kp_phi*(phi_r - phi) - P.kd_phi*phi_dot

		out = [F, tau]
		return out


	def changeCtrlType(self, ctrl_type):
		self.ctrl_type = ctrl_type

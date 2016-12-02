import numpy as np
import params as P
from controller import Controller


class WhirlybirdControllerPD(Controller):

	def __init__(self):
		super(WhirlybirdControllerPD, self).__init__()


	def getForces(self, ref_input, states):
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


class WhirlybirdControllerPID(WhirlybirdControllerPD):

    def __init__(self):
        super(WhirlybirdControllerPD, self).__init__()

        # phi
        self.phidot = 0.0
        self.int_phi = 0.0
        self.phi_d1 = P.phi0
        self.err_phi_d1 = 0.0
        # theta
        self.thetadot = 0.0
        self.int_theta = 0.0
        self.theta_d1 = P.theta0
        self.err_theta_d1 = 0.0
        # psi
        self.psidot = 0.0
        self.int_psi = 0.0
        self.psi_d1 = P.psi0
        self.err_psi_d1 = 0.0

        self.a1 = (2*P.tau - P.Ts)/(2*P.tau + P.Ts)
        self.a2 = 2.0 / (2*P.tau + P.Ts)

        self.PWM_MAX = 0.6

        self.forces = [0.0, 0.0]


    def getForces(self, ref_input, states):
        phi = states.item(0)
        theta = states.item(1)
        psi = states.item(2)
        theta_r = ref_input[0]
        psi_r = ref_input[1]

        # LONGITUDINAL CONTROL
        err = theta_r - theta
        self.thetadot = self.a1*self.thetadot + self.a2*(theta - self.theta_d1)
        self.int_theta += P.Ts/2.0 * (err + self.err_theta_d1)
        self.theta_d1 = theta
        self.err_theta_d1 = err

        Feq = (P.m1*P.l1 - P.m2*P.l2)*P.g*np.cos(theta) / P.l1
        F_unsat = P.kp_th*err - P.kd_th*self.thetadot + P.ki_th*self.int_theta + Feq
        # anti-windup?

        # LATERAL CONTROL
        # Outer loop: calculate phi_r from psi
        err = psi_r - psi
        self.psidot = self.a1*self.psidot + self.a2*(psi - self.psi_d1)
        self.int_psi += P.Ts/2.0 * (err - self.err_psi_d1)
        self.psi_d1 = psi
        self.err_psi_d1 = err

        phi_r = P.kp_psi*err - P.kd_psi*self.psidot + P.ki_psi*self.int_psi
        # anti-windup?
        
        # Inner loop: calculate tau
        err = phi_r - phi
        self.phidot = self.a1*self.phidot + self.a2*(phi - self.phi_d1)
        self.phi_d1 = phi
        self.err_phi_d1 = err

        tau = P.kp_phi*err - P.kd_phi*self.phidot

        x = 1.0/(2.0*P.km)
        ul = self.saturate(x * (F_unsat + tau/P.d), self.PWM_MAX)
        ur = self.saturate(x * (F_unsat - tau/P.d), self.PWM_MAX)

        self.forces = [ul, ur]
        return self.forces


class WhirlybirdControllerFullState(Controller):
	
	def __init__(self):
		super(WhirlybirdControllerFullState, self).__init__()

		self.phidot = 0.0
		self.thdot = 0.0
		self.psidot = 0.0
		self.phi_d1 = P.phi0
		self.th_d1 = P.theta0
		self.psi_d1 = P.psi0

		self.integrator_lon = 0.0
		self.err_lon_d1 = 0.0
		self.integrator_lat = 0.0
		self.err_lat_d1 = 0.0

		self.a1 = (2*P.tau - P.Ts) / (2*P.tau + P.Ts)
		self.a2 = 2.0 / (2*P.tau + P.Ts)
		self.Ts_half = P.Ts / 2.0

		self.PWM_MAX = 0.6

	def getForces(self, ref_input, states):
		th_r, psi_r = ref_input
		phi = states.item(0)
		theta = states.item(1)
		psi = states.item(2)

		
		self.phidot = self.a1*self.phidot + self.a2*(phi - self.phi_d1)
		self.phi_d1 = phi

		self.thdot = self.a1*self.thdot + self.a2*(theta - self.th_d1)
		self.th_d1 = theta

		self.psidot = self.a1*self.psidot + self.a2*(psi - self.psi_d1)
		self.psi_d1 = psi

		err = th_r - theta
		self.integrator_lon += self.Ts_half*(err + self.err_lon_d1)
		self.err_lon_d1 = err

		err = psi_r - psi
		self.integrator_lat += self.Ts_half*(err + self.err_lat_d1)
		self.err_lat_d1 = err

		s_lon = np.matrix([
			[theta],
			[self.thdot]
		])

		s_lat = np.matrix([
			[phi],
			[psi],
			[self.phidot],
			[self.psidot]
		])

		F = P.Feq*np.cos(theta) - P.K_lon*s_lon - P.ki_lon*self.integrator_lon
		F = F.item(0)

		tau = -P.K_lat*s_lat - P.ki_lat*self.integrator_lat
		tau = tau.item(0)

		x = 1.0/(2.0*P.km)
		ul = self.saturate(x * (F + tau/P.d), self.PWM_MAX)
		ur = self.saturate(x * (F - tau/P.d), self.PWM_MAX)

		self.forces = [ul, ur]
		return self.forces


class WhirlybirdControllerObserver(Controller):
	
	def __init__(self):
		super(WhirlybirdControllerObserver, self).__init__()
		self.xeq_lon = np.matrix([
			[P.theta0],
			[P.thetadot0]
		])
		self.xhat_lon = self.xeq_lon.copy()
		self.xeq_lat = np.matrix([
			[P.phi0],
			[P.psi0],
			[P.phidot0],
			[P.psidot0]
		])
		self.xhat_lat = self.xeq_lat.copy()

		self.F_d1 = P.Feq
		self.tau_d1 = 0.0
		self.integrator_lon = 0.0
		self.integrator_lat = 0.0
		self.err_d1_lon = 0.0
		self.err_d1_lat = 0.0

		self.PWM_MAX = 0.6

	def getForces(self, ref_input, states):
		th_r, psi_r = ref_input
		phi = states.item(0)
		th = states.item(1)
		psi = states.item(2)

		N = 10
		for i in xrange(N):
			Ts = P.Ts/N
			self.xhat_lon += Ts*(P.A_lon*(self.xhat_lon-self.xeq_lon)
			               + P.B_lon*(self.F_d1 - P.Feq)
			               + P.L_lon*([th] - P.C_lon*self.xhat_lon))

			self.xhat_lat += Ts*(P.A_lat*(self.xhat_lat-self.xeq_lat)
		                 + P.B_lat*self.tau_d1
			               + P.L_lat*([[phi],[psi]] - P.C_lat*self.xhat_lat))

		err = th_r - self.xhat_lon.item(0)
		self.integrator_lon += P.Ts/2.0 * (err+self.err_d1_lon)
		self.err_d1_lon = err

		err = psi_r - self.xhat_lat.item(1)
		self.integrator_lat += P.Ts/2.0 * (err+self.err_d1_lat)
		self.err_d1_lat = err
		
		F = P.Feq*np.cos(th)-P.K_lon*(self.xhat_lon-self.xeq_lon)-P.ki_lon*self.integrator_lon
		F = F.item(0)
		self.F_d1 = F

		tau = -P.K_lat*(self.xhat_lat-self.xeq_lat)-P.ki_lat*self.integrator_lat
		tau = tau.item(0)
		self.tau_d1 = tau

		x = 1.0/(2.0*P.km)
		ul = self.saturate(x * (F + tau/P.d), self.PWM_MAX)
		ur = self.saturate(x * (F - tau/P.d), self.PWM_MAX)

		self.forces = [ul, ur]
		return self.forces

	def getObservedStates(self):
		return self.xhat_lon.flatten().tolist()[0] + self.xhat_lat.flatten().tolist()[0]

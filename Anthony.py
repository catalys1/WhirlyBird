



Q = np.matrix([[d*(fl - fr)],
			   [l1*(fl + fr)*c_phi],
			   [l1*(fl + fr)*c_theta*s_phi + d*(fr - fl)*s_theta]]);

dP_dq = np.([[0],
			 [(m1*l1-m2*l2)*g*c_theta],
			 [0]])
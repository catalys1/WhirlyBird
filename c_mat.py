
c = np.matrix([[-thetadot**2*(Jz - Jy)s_phi*c_phi + psidot**2*(Jz - Jy)*s_phi*c_phi*c_theta**2 - phidot*psidot*c_theta*(Jx - (Jz - Jy)*(c_phi**2 - s_phi**2))],
[psidot**2*s_theta*c_theta*(-Jx + m1*l1**2 + m2*l2**2 + Jy*s_phi**2 + Jz*c_phi**2) - 2*phidot*thetadot*(Jz - Jy)*s_phi*c_phi - phidot*psidot*c_theta*(-Jx + (Jz - Jy)*(c_phi**2 - s_phi**2))],[thetadot**2*(Jz - Jy)*s_phi*c_phi*s_theta - phidot*thetadot*c_theta*(Jx + (Jz - Jy)*(c_phi**2 - s_phi**2)) - 2*phidot*psidot*(Jz - Jy)*c_theta**2*s_phi*c_phi + 2*thetadot*psidot*s_theta*c_theta*(Jx - m1*l1**2 - m2*l2**2 - Jy*s_phi**2 - Jz*c_phi**2)]])
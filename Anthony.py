


def convertForces(self, u):
	F, tau = u

	fl = F/2 + tau/(2*d)
	fr = F/2 - tau/(2*d)

	return [fl, fr]


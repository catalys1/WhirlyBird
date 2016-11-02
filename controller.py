import numpy as np

class Controller(object):

	def __init__(self):
		self.forces = [0.0]


	def saturate(self, F, limit):
		if abs(F) > limit:
			F = limit*np.sign(F)
		return F


	def getForces(self, ref_input, state):
		pass


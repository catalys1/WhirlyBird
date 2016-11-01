#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatch
import matplotlib.animation as ani
import params as P
import argparse
from WhirlybirdAnimation import WhirlybirdAnimation
from WhirlybirdDynamics import WhirlybirdDynamics
from WhirlybirdController import WhirlybirdController


parser = argparse.ArgumentParser()
parser.add_argument('inputs', nargs=2, help='Reference longitude and lateral')
args = parser.parse_args()

pitch_r, yaw_r = [float(x) for x in args.inputs]

controller = WhirlybirdController()
animator = WhirlybirdAnimation()
dynamics = WhirlybirdDynamics(controller)

t_start = 0.0
t_end = 20.0
t_Ts	= 0.01
t_elapse = 0.1
t_pause = 0.01

t = t_start


hz = 0.05
period = 1.0 / hz
t_count = t
go_up = True
go_dwn = False

inpt = []
pitch = []
yaw = []

#while t < t_end:
for _ in xrange(int(t_start), int(t_end/t_elapse)):

	if t_count < period/2.0:
		if go_up:
			pitch_r = 15 * np.pi/180.0
			go_up = False
			go_dwn = True
	else:
		if go_dwn:
			pitch_r = -pitch_r #15 * np.pi/180.0
			go_up = True
			go_dwn = False
		
	if t_count >= period:
		t_count = 0.0
	t_count += .1

	inpt.append(pitch_r)

	u = [pitch_r, pitch_r]

	plt.figure(animator.fig.number)
	plt.pause(0.001)
	t_temp = t+t_elapse
#	while t < t_temp:
	for _ in xrange(0, int(t_elapse/t_Ts)):
		dynamics.propogateDynamics(u)
		t += t_Ts
	plt.figure(animator.fig.number)
	outputs = dynamics.Outputs()
	pitch.append(outputs[1])
	yaw.append(outputs[2])
	animator.drawWhirlybird(outputs)

dynamics.Outputs()

t = np.linspace(t_start, t_end, len(inpt))
plt.figure()
plt.plot(t, inpt, 'b--')
plt.plot(t, pitch, 'r-')
plt.plot(t, yaw, 'g-')
plt.xticks(np.arange(t_start, t_end+1, 1.0))
plt.grid()
plt.title('Whirlybird Response')
plt.ylabel('Radians')
plt.xlabel('Time')
plt.annotate('Blue: Reference Input', xy=(13,0.27), color='blue')
plt.annotate('Red: Yaw angle', xy=(13,0.25), color='red')
plt.annotate('Green: Roll angle', xy=(13,0.23), color='green')
plt.show()



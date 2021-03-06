#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import params as P
import argparse
from WhirlybirdAnimation import WhirlybirdAnimation
from WhirlybirdDynamics import WhirlybirdDynamics
from WhirlybirdController import *
import signalGenerator as sigGen

parser = argparse.ArgumentParser()
parser.add_argument('-t', type=float)
args = parser.parse_args()

#controller = WhirlybirdControllerPID()
#controller = WhirlybirdControllerFullState()
controller = WhirlybirdControllerObserver()
dynamics = WhirlybirdDynamics(controller)
animator = WhirlybirdAnimation()

t_end = 20.0
if args.t: t_end = args.t
t_elapse = 0.1

hz = 0.05
pitch_r = 15 * np.pi/180.0
yaw_r = 30 * np.pi/180.0
pitch_command = sigGen.squareWave(-pitch_r, pitch_r, hz, P.Ts)
yaw_command = sigGen.squareWave(-yaw_r, yaw_r, hz, P.Ts)

pitch_ref = []
yaw_ref = []
pitch = []
yaw = []
force_left = []
force_right = []
pitch_obs = []
yaw_obs = []

plt.figure(animator.fig.number)
for _ in xrange(int(t_end/t_elapse)):
	plt.pause(0.001)
	for _ in xrange(int(t_elapse/P.Ts)):
		pitch_r = next(pitch_command)
		yaw_r = next(yaw_command)
		u = [pitch_r, yaw_r]
		dynamics.propogateDynamics(u)
	pitch_ref.append(pitch_r)
	yaw_ref.append(yaw_r)

	outputs = dynamics.Outputs()
	pitch.append(outputs[1])
	yaw.append(outputs[2])
	forces = controller.forces
	force_left.append(forces[0])
	force_right.append(forces[1])
	animator.drawWhirlybird(outputs)

	observed = controller.getObservedStates()
	pitch_obs.append(observed[0])
	yaw_obs.append(observed[3])

dynamics.Outputs()

t = np.linspace(0, t_end, len(force_left))
plt.figure(2)
plt.subplot(311)
plt.plot(t, force_left, 'b-')
plt.plot(t, force_right, 'r-')
plt.title('Left and right input forces')
plt.xticks(np.arange(0, t_end+1, 1.0))
plt.grid()
plt.ylabel('PWM')
plt.subplot(312)
plt.plot(t, pitch_ref, 'b--')
plt.plot(t, pitch, 'r-')
plt.title('Pitch angle response')
plt.xticks(np.arange(0, t_end+1, 1.0))
plt.grid()
plt.ylabel('Radians')
plt.subplot(313)
plt.plot(t, yaw_ref, 'b--')
plt.plot(t, yaw, 'r-')
plt.title('Yaw angle response')
plt.xticks(np.arange(0, t_end+1, 1.0))
plt.grid()
plt.ylabel('Radians')
plt.xlabel('Time')
# plt.annotate('Blue: Reference Input', xy=(13,0.27), color='blue')
# plt.annotate('Red: Yaw angle', xy=(13,0.25), color='red')
# plt.annotate('Green: Roll angle', xy=(13,0.23), color='green')
plt.figure(3)
plt.subplot(311)
plt.plot(t, pitch, 'r-')
plt.plot(t, pitch_obs, 'b--')
plt.grid()
plt.subplot(312)
plt.plot(t, yaw, 'r-')
plt.plot(t, yaw_obs, 'b--')
plt.grid()
plt.subplot(313)
plt.plot(t, np.array(pitch)-np.array(pitch_obs), 'r-')
plt.plot(t, np.array(yaw)-np.array(yaw_obs), 'b-')
plt.grid()
plt.show()



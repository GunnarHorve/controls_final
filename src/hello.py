#!/usr/bin/env python
import pickle
import numpy as np
import matplotlib.pyplot as plt


# Tuple order:  Position, Velocity, Torque, Time
Data = pickle.load(open('save.p'))
time = np.array(Data[3])
Data = (np.array(Data[0]), np.array(Data[1]), np.array(Data[2]))

# hard-coded, us-specific data
legend  = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
ylabels = ['position (rad)', 'velocity (rad/s)', 'torque (N/m)']
titles = ['Position vs. Time', 'Velocity vs Time', 'Torque vs. Time']
joints = range(len(legend))
joints = [6]


# create plot references
figures = [plt.figure(i) for i in range(len(ylabels))]
axes = [fig.add_subplot(111) for fig in figures]

for plot_type in range(len(axes)):
	# plot graphs
	for joint in joints:
		axes[plot_type].plot(time, Data[plot_type][:,joint])

	# label graphs	
	axes[plot_type].legend([legend[i] for i in joints], loc = 'upper right')
	axes[plot_type].set_xlabel('time (s)')
	axes[plot_type].set_ylabel(ylabels[plot_type])
	axes[plot_type].set_title(titles[plot_type])
	figures[plot_type].show()


# prevent program close
while(True):
	print("don't close me pl0x")
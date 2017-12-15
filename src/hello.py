#!/usr/bin/env python
import pickle
import numpy as np
import matplotlib.pyplot as plt
import os

# Handle figure saving to .jpg files
filename = 'save'
if not os.path.exists(filename):
    os.makedirs(filename)

# Tuple order:  Position, Velocity, Torque, Time, Desired Position, Desired Velocity
Data = pickle.load(open(filename + '.p'))
time = np.array(Data[3])
Data = (np.array(Data[0]), np.array(Data[1]), np.array(Data[2]), np.array(Data[4]), np.array(Data[5]))

# hard-coded, us-specific data
legend  = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

ylabels = ['position (rad)', 'velocity (rad/s)', 'torque (N*m)', 'position (rad)', 'velocity (rad/s)']
titles = ['Position vs. Time', 'Velocity vs Time', 'Torque vs. Time', 'Desired Position vs. Time', 'Desired Velocity vs. Time']
joints = range(len(legend))
# joints = [2]


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

	figures[plot_type].savefig(filename + '/' + titles[plot_type] + '.jpg')


# # error plot (less nice && pretty ;-;
# fig  = plt.figure(len(ylabels))
# ax = fig.add_subplot(111)

# pos = Data[0]
# pos_des = Data[3]
# for i in range(len(joints)):
# 	ax.plot(time, pos_des[:,i] - pos[:,i])
# 	print(pos_des[:,i] - pos[:,i])
# fig.show()


# prevent program close
while(True):#
	print("don't close me pl0x")
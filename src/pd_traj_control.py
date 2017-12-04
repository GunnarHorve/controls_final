#!/usr/bin/env python

"""
Baxter RSDK Joint Torque Example: joint springs
"""
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty
import baxter_interface

class JointController(object):
    """
    Virtual Joint Controller class for torque control.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server
    """
    def __init__(self, limb):
        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self._kp = {'left_s0':10., 'left_s1':15., 'left_e0':5.,
                         'left_e1':5., 'left_w0':3., 'left_w1':2., 'left_w2':1.5}
        self._kd = {'left_s0':.1, 'left_s1':.1, 'left_e0':.1,
                         'left_e1':.1, 'left_w0':.1, 'left_w1':.1, 'left_w2':.1}
        self._des_pos = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(True)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        # calculate current forces
        for joint in self._des_pos.keys():
            # spring portion
            cmd[joint] = self._kp[joint] * (self._des_pos[joint] -
                                                   cur_pos[joint])
            # damping portion
            cmd[joint] -= self._kd[joint] * cur_vel[joint]

        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def start_control(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        print("starting control")
        # record initial joint angles
        self._des_pos = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()


def main():
    """Much of this code is cited from Baxter's Joint Torque Example

    Moves the specified limb to a neutral location and enters
    torque control mode, attaching virtual springs (Hooke's Law)
    to each joint maintaining the start position.
    """

    print("Initializing node... ")
    rospy.init_node("controls_final_left_arm")

    jc = JointController("left",)           # instantiate control object
    rospy.on_shutdown(jc.clean_shutdown)    # register shutdown callback
    jc.move_to_neutral()                    # start from neutral arm position
    jc.start_control()                      # turn on control law


if __name__ == "__main__":
    main()

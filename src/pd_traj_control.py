#!/usr/bin/env python

"""
Baxter RSDK Joint Torque Example: joint springs
"""
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty
import baxter_interface
from baxter_interface import CHECK_VERSION

class JointController(object):
    """
    Virtual Joint Springs class for torque control.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointController class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb):
        

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")


        # assign appropriate joint parameters to damping and springs values
        default_spring = [10.0, 15.0, 5.0, 5.0, 3.0, 2.0, 1.5]
        default_damping = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        i = 0

        for joint in self._limb.joint_names():
            self._springs[joint] = default_spring[i]
            self._damping[joint] = default_damping[i]
            i = i+1



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
        for joint in self._start_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                                                   cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]

        print(cmd)

        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def attach_springs(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

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
    rospy.init_node("rsdk_joint_torque_springs_left")
    

    js = JointController("left") # instatiation control object
    rospy.on_shutdown(js.clean_shutdown)       # register shutdown callback
    js.move_to_neutral()                       # start from neutral arm position
    js.attach_springs()                        # turn on control law


if __name__ == "__main__":
    main()

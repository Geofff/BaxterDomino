
import argparse
import sys

import rospy

import baxter_interface


class Waypoints(object):
    def __init__(self, limb, speed, accuracy):
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)
        self._gripper = baxter_interface.Gripper(self._arm)
        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Recorded waypoints
        self._waypoints = list()

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Create Navigator I/O
        self._navigator_io = baxter_interface.Navigator(self._arm)

    def _record_waypoint(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            self._waypoints.append(self._limb.joint_angles())

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            self._is_recording = False

    def record(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new joint "
        "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press
        self._navigator_io.button0_changed.connect(self._record_waypoint)
        # Navigator Rethink button press
        self._navigator_io.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)

        # We are now done with the navigator I/O signals, disconnecting them
        self._navigator_io.button0_changed.disconnect(self._record_waypoint)
        self._navigator_io.button2_changed.disconnect(self._stop_recording)
    def _find_approach(self, pose, offset):
        ikreq = SolvePositionIKRequest()
        pose['position'] = Point(x=pose['position'][0], y = pose['position'][1], z = pose['position'][2] + offset)
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ikreq.pose_stamp.append(pose_req)
        resp=self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def playback(self):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Playback Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        if len(self._waypoints) < 2:
            print("Not enough waypoints. Exiting")
            return
        # Only Perform one loop

        waypointApproach = []
        for waypoint in self._waypoints:
            waypointApproach.append(self._find_approach(waypoint, 5))
            
        homeWaypoint = self._waypoints[0]
        homeWaypointApproach = waypointApproach[0]

        self._gripper.open()
        rospy.sleep(1.0)
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(homeWaypointApproach, timeout=20.0, threshold=0.01745)
        self._limb.set_joint_position_speed(0.3)
        self._limb.move_to_joint_positions(homeWaypoint, timeout=20.0, threshold=0.003491)
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(homeWaypointApproach, timeout=20.0, threshold=0.034906)
        rospy.sleep(1.0)
        self._gripper.close()
        rospy.sleep(1.0)


        # Loop through rest of waypoints
        for waypoint in self._waypoints[1:]:
            waypointApproach = waypointApproach[self._waypoints.index(waypoint)]
            if rospy.is_shutdown():
                break
            # Move to domino placement
            self._limb.set_joint_position_speed(0.8)
            self._limb.move_to_joint_positions(waypointApproach, timeout=20.0, threshold=0.01745)
            self._limb.set_joint_position_speed(0.3)
            self._limb.move_to_joint_positions(waypoint, timeout=20.0, threshold=0.00349)

            # Drop off domino
            rospy.sleep(1.0)
            self._gripper.open()
            rospy.sleep(1.0)
            self._limb.move_to_joint_positions(waypointApproach, timeout=20.0, threshold=0.034906)


            # Head to home
            self._limb.set_joint_position_speed(0.8)
            self._limb.move_to_joint_positions(homeWaypointApproach, timeout=20.0, threshold=0.01745)
            self._limb.set_joint_position_speed(0.3)
            self._limb.move_to_joint_positions(homeWaypoint, timeout=20.0, threshold=0.003491)

            # Grab next domino
            rospy.sleep(1.0)
            self._gripper.close()
            rospy.sleep(1.0)

            # Step away
            self._limb.move_to_joint_positions(homeWaypointApproach, timeout=20.0, threshold=0.034906)



        self._limb.set_joint_position_speed(0.3)

    def clean_shutdown(self):
        print("\nExiting example...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True


def main():
    """RSDK Joint Position Waypoints Example

    Records joint positions each time the navigator 'OK/wheel'
    button is pressed.
    Upon pressing the navigator 'Rethink' button, the recorded joint positions
    will begin playing back in a loop.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='limb to record/playback waypoints'
    )
    parser.add_argument(
        '-s', '--speed', default=0.3, type=float,
        help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
    )
    parser.add_argument(
        '-a', '--accuracy',
        default=baxter_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
        help='joint position accuracy (rad) at which waypoints must achieve'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_waypoints_%s" % (args.limb,))

    waypoints = Waypoints(args.limb, args.speed, args.accuracy)

    # Register clean shutdown
    rospy.on_shutdown(waypoints.clean_shutdown)

    # Begin example program
    waypoints.record()
    waypoints.playback()

if __name__ == '__main__':
    main()
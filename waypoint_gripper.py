
import argparse
import copy
import sys

import rospy

import baxter_interface
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)


class Waypoints(object):
    def __init__(self, limb, speed, accuracy, distance):
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)
        self._gripper = baxter_interface.Gripper(self._arm)
        self._ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(self._ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()
	self._hover_distance = distance

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
	    
	    self._waypoints.append(self._limb.endpoint_pose())

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
    def _find_approach(self, current_pose, offset):
	print("Finding approach with offset %d" % offset)
        ikreq = SolvePositionIKRequest()
	pose = copy.deepcopy(current_pose)
	try:
        	pose['position'] = Point(x=pose['position'][0], y = pose['position'][1], z = pose['position'][2] + offset)
	except Exception:
		pose['position'] = Point(x=pose['position'].x, y=pose['position'].y, z=pose['position'].z+offset)
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']
	pose = approach_pose
	print(pose)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=pose)
        ikreq.pose_stamp.append(pose_req)
	rospy.wait_for_service(self._ik_srv, 5.0)
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
	waypointJP = []
        for waypoint in self._waypoints:
            waypointApproach.append(self._find_approach(waypoint, self._hover_distance))
            waypointJP.append(self._find_approach(waypoint, 0))
            
        homeWaypoint = waypointJP[0]
        homeWaypointApproach = waypointApproach[0]
	print("%d Waypoints\n%d JPs\n%d Approaches"%(len(self._waypoints), len(waypointJP), len(waypointApproach)))
        self._gripper.open()
        rospy.sleep(1.0)
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(homeWaypointApproach)
        self._limb.set_joint_position_speed(0.3)
        self._limb.move_to_joint_positions(homeWaypoint)
        rospy.sleep(1.0)
        self._gripper.close()
        rospy.sleep(1.0)
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(homeWaypointApproach)

	print("=== Waypoints ===")
	print(waypointJP)
	print("=== Approaches ===")
	print(waypointApproach)
        # Loop through rest of waypoints
	for i in range(len(waypointJP[1:])-1):
	    waypoint = waypointJP[i+1]
	    waypointA = waypointApproach[i+1]
            if rospy.is_shutdown():
                break
            # Move to domino placement
	    print("Moving ti Approahc point")
            self._limb.set_joint_position_speed(0.8)
            self._limb.move_to_joint_positions(waypointA)
	    print("Moving to real point")
            self._limb.set_joint_position_speed(0.3)
            self._limb.move_to_joint_positions(waypoint)

            # Drop off domino
            rospy.sleep(1.0)
            self._gripper.open()
	    print("About to move to approach point")
            rospy.sleep(1.0)
            self._limb.move_to_joint_positions(waypointA)


            # Head to home
	    print("Moving to home approach")
            self._limb.set_joint_position_speed(0.8)
            self._limb.move_to_joint_positions(homeWaypointApproach)
	    print("Moving to home")
            self._limb.set_joint_position_speed(0.3)
            self._limb.move_to_joint_positions(homeWaypoint)

            # Grab next domino
            rospy.sleep(1.0)
            self._gripper.close()
	    print("Moving to home approach")
            rospy.sleep(1.0)

            # Step away
            self._limb.move_to_joint_positions(homeWaypointApproach)



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
    parser.add_argument(
	'-d', '--distance',
	default=0.15, type=float,
	help='Distance above domino to hover'
    )
    parser.add_argument(
	'-n', '--loops',
	default=1, type=int,
	help="Number of times to loop the program"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_waypoints_%s" % (args.limb,))

    waypoints = Waypoints(args.limb, args.speed, args.accuracy, args.distance)

    # Register clean shutdown
    rospy.on_shutdown(waypoints.clean_shutdown)

    # Begin example program
    waypoints.record()
    waypoints.playback()

if __name__ == '__main__':
    main()

from detector import image_converter
import argparse
import copy
import sys
import math

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
    def __init__(self, limb, distance, loops, calibrate):
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)
        self._gripper = baxter_interface.Gripper(self._arm)
        self._ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(self._ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()
        self._hover_distance = distance
        self._num_loops = loops


        self._domino_source_approach = None
        self._domino_source = None
        self._domino_source_jp = None
        self._calibration_point = None
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

        # Calibrate Gripper
	if (calibrate):
		self._gripper.calibrate()

        if not (self._gripper.calibrated() or self._gripper.calibrate()):
            rospy.logwarn("%s (%s) calibration failed.",
                          self._gripper.name.capitalize(),
                          self._gripper.type())

        # Create Navigator I/O
        self._navigator_io = baxter_interface.Navigator(self._arm)
        self._overhead_orientation = Quaternion(
            x=-0.0249590815779,
            y=0.999649402929,
            z=0.00737916180073,
            w=0.00486450832011)

    def _record_waypoint(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            if self._domino_source_jp is None:
            	self._domino_source_jp = self._limb.endpoint_pose()
	    	print("Domino source set")
            elif self._calibration_point is None:
                self._calibration_point = self._limb.endpoint_pose()
                print("Calibrated to point")  
            else:
	    	print("Added waypoint")
            	self._waypoints.append(self._limb.endpoint_pose())

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            self._is_recording = False

    def generatePath(self):
	waypointApproach = []
	waypointJP = []
	for i in range(self._numDominos):
            waypointApproach.append(self._find_approach(self._waypoints[0], self._hover_distance,-0.05*i))
            waypointJP.append(self._find_approach(self._waypoints[0], 0,-0.05*i))
	return waypointApproach, waypointJP

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
    def _find_approach(self, current_pose, offset, offsetx, offsety=0, ang=0):
        print("Finding approach for")
        print(current_pose)
        print("Finding approach with offset %d" % offset)
        ikreq = SolvePositionIKRequest()
        pose = copy.deepcopy(current_pose)
        try:
            pose['position'] = Point(
                x=pose['position'][0] + offsetx,
                y=pose['position'][1] + offety,
                z=pose['position'][2] + offset)
        except Exception:
            pose['position'] = Point(
                x=pose['position'].x + offsetx,
                y=pose['position'].y + offsety,
                z=pose['position'].z+offset)
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = self._overhead_orientation
        if ang == 0:
	        approach_pose.orientation.x = pose['orientation'].x
        else:
	        approach_pose.orientation.x = ang
                
        pose = approach_pose
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=pose)
        ikreq.pose_stamp.append(pose_req)
        rospy.wait_for_service(self._ik_srv, 5.0)
        resp = self._iksvc(ikreq)
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
        self._limb.set_joint_position_speed(0.8)

        if len(self._waypoints) < 1:
            print("Not enough waypoints. Exiting")
            return
        # Only Perform one loop
	self._numDominos = 3
	waypointApproach, waypointJP = self.generatePath()
	self._knock_approach = self._find_approach(self._waypoints[0], 0.05, 0.05)
	print(self._knock_approach)
	self._knock_approach['left_w2']+=3.14159/2
	self._knock = self._find_approach(self._waypoints[0], 0.05, -0.05)
	self._knock['left_w2']+=3.14159/2
        self._domino_source_approach = self._find_approach(self._domino_source_jp, self._hover_distance,0)
	self._domino_source = self._find_approach(self._domino_source_jp, 0,0)
        print("%d Waypoints\n%d JPs\n%d Approaches"%(len(self._waypoints), len(waypointJP), len(waypointApproach)))
        self._gripper.open()
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(self._domino_source_approach)
        self._limb.set_joint_position_speed(0.3)
        self._limb.move_to_joint_positions(self._domino_source)
        self._gripper.close()
        self._limb.set_joint_position_speed(0.8)
        self._limb.move_to_joint_positions(self._domino_source_approach)

        print("=== Waypoints ===")
        print(waypointJP)
        print("=== Approaches ===")
        print(waypointApproach)
        for j in range(self._num_loops):
                # Loop through rest of waypoints
            for i in range(len(waypointJP)):
                waypoint = waypointJP[i]
                waypointA = waypointApproach[i]
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
                self._limb.move_to_joint_positions(self._domino_source_approach)
		if (i != len(waypointJP)-1):
                	print("Moving to home")
                	self._limb.set_joint_position_speed(0.3)
                	self._limb.move_to_joint_positions(self._domino_source)
        
       		        # Grab next domino
                	rospy.sleep(1.0)
                	self._gripper.close()
                	print("Moving to home approach")
                	rospy.sleep(1.0)

                	# Step away
                	self._limb.move_to_joint_positions(self._domino_source_approach)
		else:
			self._gripper.close()



            self._limb.set_joint_position_speed(0.3)
	    # Knock over all dominos
	self._limb.move_to_joint_positions(self._knock_approach)
	self._limb.move_to_joint_positions(self._knock)
	self._limb.move_to_joint_positions(self._domino_source_approach)
		

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
    parser.add_argument(
	'-c', '--calibrate', action='store_true', default=False, help='Recalibrate grippers')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='limb to record/playback waypoints'
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

    rospy.init_node('image_converter', anonymous=True)
    waypoints = Waypoints(args.limb, args.distance, args.loops, args.calibrate)


    

    # Begin example program
    waypoints.record()
    # Temp code to pickup placed dominos
     

    ic = image_converter()
    print("Initialising...")
    print(waypoints._calibration_point)
    try:
        while 1:
                domino = ic.getNextDomino()
                if len(domino) != 3:
                        continue
                camRot = -waypoints._calibration_point['orientation'].x
                dx = math.cos(camRot+3.1415+math.radians(domino[1]))*domino[0]/1000
                dy = math.sin(camRot+3.1415+math.radians(domino[1]))*domino[0]/1000
                domAng = math.radians(domino[2])+camRot+3.1415
                approach = waypoints._find_approach(waypoints._calibration_point, args.distance, dx, dy, domAng)
                pos = waypoints._find_approach(waypoints._calibration_point, 0, dx, dy, domAng)
                print(approach)        
                waypoints._limb.move_to_joint_positions(approach)
                waypoints._limb.move_to_joint_positions(pos)
                print(domino)
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

    waypoints.playback()

if __name__ == '__main__':
    main()

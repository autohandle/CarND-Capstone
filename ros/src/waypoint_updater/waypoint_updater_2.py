#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped,  Quaternion, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Header

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

def stampNewLane(theNewLane, theSequenceNumber, theOldLane):
    #rospy.loginfo("stampNewLane-theNewLane["+str(theSequenceNumber)+"]:\n"+str(theNewLane)
    #    +"\ntheSequenceNumber:"+str(theSequenceNumber))
    stampMessageWithHeader(theNewLane, theSequenceNumber, theOldLane.header.frame_id)

def stampWaypointTwist(theWaypoint, theSequenceNumber, theFrameId):
    #rospy.loginfo("stampWaypointTwist-theWaypoint["+str(theSequenceNumber)+"]:\n"+str(theWaypoint))
    stampMessageWithHeader(theWaypoint.twist, theSequenceNumber, theFrameId)

def stampMessageWithHeader(theMessageWithHeader, theSequenceNumber, theFrameId):
    #rospy.loginfo("stampMessageWithHeader-theMessageWithHeader["+str(theSequenceNumber)+"]:\n"+str(theMessageWithHeader))
    stampHeader(theMessageWithHeader.header, theSequenceNumber, theFrameId)

def stampHeader(theHeader, theSequenceNumber, theFrameId):
    # https://answers.ros.org/question/60209/what-is-the-proper-way-to-create-a-header-with-python/
    #srospy.loginfo("stampHeader-theHeader["+str(theSequenceNumber)+"]:\n"+str(theHeader))
    theHeader.stamp=rospy.Time.now()
    theHeader.seq=theSequenceNumber
    theHeader.frame_id=theFrameId

HOURSPERMINUTE = 1./60.
MINUTESPERSECOND = 1./60.
HOURSPERSECOND = HOURSPERMINUTE*MINUTESPERSECOND

FEETPERMILE= 5280.
METERSPERFOOT = .3048
FEETPERMETER = 1./METERSPERFOOT
MILESPERHOURTOMETERSPERSECOND = HOURSPERSECOND*FEETPERMILE*METERSPERFOOT;

def milesPerHourToMetersPerSecond(theMilesPerHour):
    return theMilesPerHour*MILESPERHOURTOMETERSPERSECOND;

def metersPerSecondToMilesPerHour(theMetersPerSecond):
    return theMetersPerSecond*1./MILESPERHOURTOMETERSPERSECOND;

class WaypointUpdater2(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.traffic_waypoints = None
        self.obstacle_waypoints = None

        #self.loop()
        rospy.spin()

    def get_yaw(self, orientation):
        """
        Compute yaw from orientation, which is in Quaternion.
        """
        # orientation = msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
        yaw = euler[2]
        return yaw

    def get_closest_waypoint(self, pose):
        closest_wp_idx = None
        closest_wp_dist = 10 ** 10  # some really big distance.
        yaw = self.get_yaw(pose.orientation)

        if not self.base_waypoints or len(self.base_waypoints.waypoints) == 0:
            return closest_wp_idx, closest_wp_dist

        # Compute the waypoints ahead of the current_pose
        for i in range(len(self.base_waypoints.waypoints)):
            wp_pos = self.base_waypoints.waypoints[i].pose.pose.position
            wp_x, wp_y, _ = self.convert_coord(
                (pose.position.x, pose.position.y, yaw),
                (wp_pos.x, wp_pos.y, 0)
            )
            dist = math.sqrt((wp_x-pose.position.x)**2 + (wp_y-pose.position.y)**2)
            if wp_x > 0 and dist < closest_wp_dist:
                closest_wp_dist = dist
                closest_wp_idx = i
        return closest_wp_idx, closest_wp_dist

    WAYPOINTVELOCITYPLACEHOLDER=milesPerHourToMetersPerSecond(30.);# 30 mph
    def createWaypoints(self, firstWaypointIndex):
        waypoints=[]
        waypointSequenceNumber=0
        for wp in range(firstWaypointIndex, firstWaypointIndex+LOOKAHEAD_WPS):
            # do i need to have a circular range?
            # waypoint=>styx_msgs.Waypoint
            self.set_waypoint_velocity(self.base_waypoints.waypoints, wp, self.WAYPOINTVELOCITYPLACEHOLDER)
            waypoints.append(self.base_waypoints.waypoints[wp])
            # stampWaypoint(theWaypoint, theSequenceNumber, theFrameId):
            waypointSequenceNumber+=1
            stampWaypointTwist(waypoints[-1], waypointSequenceNumber, self.base_waypoints.header.frame_id)
        return waypoints


    sequenceNumberForFinalWaypoints=0
    def loop(self):
        """
        Loop to publish the waypoints and recommended speed
        :return: None
        """
        # limit publishing rate (uses rate.sleep later)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rate.sleep()

            # Make sure we've got valid data before publishing
            if not self.pose or not self.base_waypoints:
                rospy.logwarn("waiting for all data..")
                continue

            closest_wp_idx, _ = self.get_closest_waypoint(self.pose.pose)

            if closest_wp_idx:
                lane = Lane()
                self.sequenceNumberForFinalWaypoints+=1
                stampNewLane(lane, self.sequenceNumberForFinalWaypoints, self.base_waypoints)

                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                #lane.waypoints.append(self.base_waypoints.waypoints[closest_wp_idx])
                lane.waypoints=self.createWaypoints(closest_wp_idx)
                rospy.loginfo("WaypointUpdater.loop\nlane.waypoints[0]:\n"+str(lane.waypoints[0])
                    +"\n, lane.waypoints["+str(len(lane.waypoints)-1)+"]:\n"+str(lane.waypoints[-1]))

                # publish the waypoints
                self.final_waypoints_pub.publish(lane)

    def create_pose(self, x, y, z, yaw=0.):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0., 0., math.pi * yaw/180.)
        pose.pose.orientation = Quaternion(*q)

        return pose

    last_closest_wp_idx=-1
    closestRepeatCounter=0;
    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.logerr(msg)
        # pose:
        #   position:
        #     x: 1131.23
        #     y: 1183.27
        #     z: 0.1034537
        #
        #   orientation:
        #     x: 0.0
        #     y: 0.0
        #     z: 0.0436139994303
        #     w: 0.99904845681
        self.pose = msg
        if not self.pose or not self.base_waypoints:
            rospy.logwarn("WaypointUpdater.pose_cb-waiting for all data..")
            #continue
            return

        #closest_wp_idx, _ = self.get_closest_waypoint(self.pose.pose)
        closest_wp_idx=self.findWaypointInFront(self.pose)
        self.closestRepeatCounter+=1

        if (self.closestRepeatCounter%100==0):
            rospy.loginfo("WaypointUpdater.pose_cb-closest_wp_idx:"+str(closest_wp_idx)
            +", closestRepeatCounter:"+str(self.closestRepeatCounter));
        if closest_wp_idx and (self.last_closest_wp_idx!=closest_wp_idx):

            rospy.loginfo("WaypointUpdater.pose_cb-closest_wp_idx:"+str(closest_wp_idx)
                +", closestRepeatCounter:"+str(self.closestRepeatCounter));
            self.last_closest_wp_idx=closest_wp_idx
            self.closestRepeatCounter=0;

            lane = Lane()
            self.sequenceNumberForFinalWaypoints+=1
            stampNewLane(lane, self.sequenceNumberForFinalWaypoints, self.base_waypoints)

            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            #lane.waypoints.append(self.base_waypoints.waypoints[closest_wp_idx])
            lane.waypoints=self.createWaypoints(closest_wp_idx)
            rospy.loginfo("WaypointUpdater.pose_cb"
                +"\nlane.waypoints[0]:\n"+str(lane.waypoints[0])
                +"\nlane.waypoints["+str(len(lane.waypoints)-1)+"]:\n"+str(lane.waypoints[-1]))

            # publish the waypoints
            self.final_waypoints_pub.publish(lane)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoints = msg
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_waypoints = msg
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def convert_coord(self, (ref_x, ref_y, ref_yaw), (pt_x, pt_y, pt_yaw)):
        shift_x = pt_x - ref_x
        shift_y = pt_y - ref_y
        cos_yaw = math.cos(ref_yaw)
        sin_yaw = math.sin(ref_yaw)

        new_x = cos_yaw * shift_x + sin_yaw * shift_y
        new_y = -sin_yaw * shift_x + cos_yaw * shift_y
        new_yaw = math.fmod(pt_yaw - ref_yaw, math.pi*2)
        return new_x, new_y, new_yaw

    def findWaypointInFront(self, vehiclePoseStamped):
        vehiclePosition=vehiclePoseStamped.pose.position
        vehicleYaw=self.get_yaw(vehiclePoseStamped.pose.orientation)
        return self.findFirstWaypointInFront(vehiclePosition, vehicleYaw, self.base_waypoints.waypoints)

    def findFirstWaypointInFront(self, vehiclePosition, vehicleYaw, waypoints):
        for wp in range(0, len(waypoints)):
            waypointPosition=waypoints[wp].pose.pose.position
            isInFront=self.isWaypointInFront(vehiclePosition, vehicleYaw, waypointPosition)
            if isInFront :
                return wp
        return -1

    def isWaypointInFront(self, vehiclePosition, vehicleYaw, wayPointPosition):
        waypointTranslatedToOrigin=[wayPointPosition.x-vehiclePosition.x, wayPointPosition.y-vehiclePosition.y]
        waypointRotatededAtOrigin=[(waypointTranslatedToOrigin[0] * math.cos(-vehicleYaw) - waypointTranslatedToOrigin[1]*math.sin(-vehicleYaw)),
            (waypointTranslatedToOrigin[0] * math.sin(-vehicleYaw) + waypointTranslatedToOrigin[1]*math.cos(-vehicleYaw))]
       # waypoint/vehicle translated & rotated so that vehicle is at 0,0 and pointing in +x direction
        # if waypoint is in -x direction -> it is behind the vehicle
        if (waypointRotatededAtOrigin[0]<0) :
            return False
        # waypoint is in the +x,+y or +x,-y quadrant
        else:
            return True 

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

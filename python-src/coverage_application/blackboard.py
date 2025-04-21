import json
import os
import rospy
from geometry_msgs.msg import PoseStamped
import py_trees
import enum

class CoverageBlackboard(py_trees.blackboard.Blackboard):

    ROBOT1= 0
    ROBOT2= 1

    def __init__(self, coverage_pose_file=None):
        super(CoverageBlackboard, self).__init__()

        # Default path
        if coverage_pose_file is None:
            coverage_pose_file = rospy.get_param("~coverage_pose_file",
                "/catkin_ws/src/coverage_application/config/coverage_poses.json")

        self.coverage_poses = self._load_coverage_poses(coverage_pose_file)
        # Print all coverage poses in the command line
        for i, pose in enumerate(self.coverage_poses):
            rospy.loginfo(f"[Blackboard] Coverage pose {i}: {pose}")

        # All poses are not visited at the beginning
        self.visited_poses = [False] * len(self.coverage_poses)
        self.visiting_poses_index = []

        # Battery level 
        self.battery_level = [100.0, 100.0]  # 2 robots
        self.battery_pose = self._load_battery_pose(coverage_pose_file)


    def _load_coverage_poses(self, path):
        if not os.path.exists(path):
            rospy.logwarn(f"[Blackboard] Coverage poses file not found: {path}")
            return []

        with open(path, 'r') as f:
            data = json.load(f)

        poses = []
        for item in data['goals']['poses']:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = item['position']
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = item['orientation']
            poses.append(pose)


        
        rospy.loginfo(f"[Blackboard] Loaded {len(poses)} coverage poses.")
        return poses

    def all_poses_visited(self):
        # Check if all poses have been visited
        # log the visited poses
        rospy.loginfo(f"[Blackboard] Visited poses: {self.visited_poses}")
        return all(self.visited_poses)


    def get_next_pose_and_index(self):
        # Find the first unvisited pose
        for i, visited in enumerate(self.visited_poses):
            if not visited:
                if i not in self.visiting_poses_index:
                    return self.coverage_poses[i], i 
        return None
  

    def _load_battery_pose(self, path):
        if not os.path.exists(path):
            rospy.logwarn(f"[Blackboard] Battery pose file not found: {path}")
            return None
        with open(path, 'r') as f:
            data = json.load(f)

        battery_pose = PoseStamped()
        battery_pose.header.frame_id = "map"
        battery_pose.pose.position.x, battery_pose.pose.position.y, battery_pose.pose.position.z = data['battery_charger']['position']
        battery_pose.pose.orientation.x, battery_pose.pose.orientation.y, battery_pose.pose.orientation.z, battery_pose.pose.orientation.w = data['battery_charger']['orientation']
        return battery_pose

    def reset(self):
        self.visited_poses = [False] * len(self.coverage_poses)
        self.battery_level = 100.0

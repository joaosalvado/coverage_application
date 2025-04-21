import rospy
import py_trees
import py_trees_ros.actions
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from coverage_application.blackboard import CoverageBlackboard
from visualization_msgs.msg import Marker, MarkerArray

class MoveToNextPoseClient(py_trees_ros.actions.ActionClient):
    def __init__(self, name="MoveNextToPose", action_namespace="/robot1/move_base", robot_id=0):
        super(MoveToNextPoseClient, self).__init__(
            name=name,
            action_spec=MoveBaseAction,
            action_namespace=action_namespace
        )
        self.blackboard = CoverageBlackboard()
        self.robot_id = robot_id

    def initialise(self):
        super().initialise()
        rospy.loginfo(f"[{self.name}] Initialising MoveToNextPoseClient")
        
        if self.blackboard.get_next_pose_and_index() is None:
            rospy.loginfo(f"[{self.name}] No more poses to cover")
            return py_trees.common.Status.FAILURE
        pose, self.pose_index = self.blackboard.get_next_pose_and_index()
        self.blackboard.visiting_poses_index.append(self.pose_index)

        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose = pose

    def terminate(self, new_status):
        super().terminate(new_status)
        if new_status == py_trees.common.Status.SUCCESS:
            #check if pose_Index is in visiting_poses_index
            if self.pose_index in self.blackboard.visiting_poses_index:
                self.blackboard.visiting_poses_index.remove(self.pose_index)
            self.blackboard.visited_poses[self.pose_index] = True
            self.blackboard.battery_level[self.robot_id] -= 30.0

class HasPosesToCover(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckAllGoals"):
        super(HasPosesToCover, self).__init__(name=name)
        self.blackboard = CoverageBlackboard()

    def update(self):
        rospy.sleep(0.2)
        rospy.loginfo(f"[visited_poses] {self.blackboard.visited_poses}")
        if self.blackboard.all_poses_visited():
            #self.blackboard.reset()
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS


class MockTask(py_trees.behaviour.Behaviour):
    def __init__(self, name="MockTask", robot_id=0,  duration=3.0):
        super(MockTask, self).__init__(name)
        self.blackboard = CoverageBlackboard()
        self.duration = rospy.Duration(duration)
        self.start_time = None
        self.robot_id = robot_id

    def initialise(self):
        super(MockTask, self).initialise()
        self.start_time = rospy.Time.now()

    def update(self):
        elapsed = rospy.Time.now() - self.start_time
        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.blackboard.battery_level[self.robot_id] -= 30.0



class HasBattery(py_trees.behaviour.Behaviour):
    def __init__(self, name="HasBattery", robot_id=0, low_battery_threshold=20.0):
        super(HasBattery, self).__init__(name=name)
        self.blackboard = CoverageBlackboard()
        self.robot_id = robot_id
        self.low_battery_threshold = low_battery_threshold

    def initialise(self):
        # print battery level
        super(HasBattery, self).initialise()
        rospy.loginfo(f"[{self.name}] Battery level: {self.blackboard.battery_level}")

    def update(self):
        rospy.sleep(0.2)
        if self.blackboard.battery_level[self.robot_id] > self.low_battery_threshold:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class MoveToFillBattery(py_trees_ros.actions.ActionClient):
    def __init__(self, name="MoveToFillBattery", action_namespace="/robot1/move_base", robot_id=0):
        super(MoveToFillBattery, self).__init__(
            name=name,
            action_spec=MoveBaseAction,
            action_namespace=action_namespace
        )
        self.blackboard = CoverageBlackboard()
        self.robot_id = robot_id

    def initialise(self):
        super().initialise()

        pose = self.blackboard.battery_pose
        rospy.loginfo(f"PoseStamped:\n{pose}")

        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose = pose

    def terminate(self, new_status):
        super().terminate(new_status)
        if new_status == py_trees.common.Status.SUCCESS:
            self.blackboard.battery_level[self.robot_id] = 100.0


class PublishCoverageMarkers(py_trees.behaviour.Behaviour):
    def __init__(self, name="PublishCoverageMarkers"):
        super().__init__(name)
        self.publisher = rospy.Publisher("coverage_markers", MarkerArray, queue_size=10)
        self.blackboard = CoverageBlackboard()

    def update(self):
        marker_array = MarkerArray()
        now = rospy.Time.now()

        for i, pose in enumerate(self.blackboard.coverage_poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "coverage"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2

            #visited = i in getattr(self.blackboard, "visited_indices", [])
            if self.blackboard.visited_poses[i]:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i in self.blackboard.visiting_poses_index:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            marker.color.a = 1.0
            marker_array.markers.append(marker)

        
        # Battery level marker
        battery_marker = Marker()
        battery_marker.header.frame_id = "map"
        battery_marker.header.stamp = now
        battery_marker.ns = "coverage"
        battery_marker.id = len(self.blackboard.coverage_poses)
        battery_marker.type = Marker.CYLINDER
        battery_marker.action = Marker.ADD
        battery_marker.pose = self.blackboard.battery_pose.pose
        battery_marker.scale.x = battery_marker.scale.y = battery_marker.scale.z = 0.2
        battery_marker.color.r = 0.0
        battery_marker.color.g = 0.0
        battery_marker.color.b = 1.0
        battery_marker.color.a = 1.0
        marker_array.markers.append(battery_marker)

        # Robot battery
        for i in [0, 1]:
            level = self.blackboard.battery_level[i]
            robot_name = f"robot{i+1}"
            marker = Marker()
            marker.header.frame_id = f"robot{i+1}/base_footprint" 
            marker.header.stamp = rospy.Time.now()
            marker.ns = "battery_levels"
            marker.id = 20 + i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            # Position text slightly above robot
            marker.pose = Pose()
            marker.pose.position.z += 0.5
            marker.scale.z = 0.3  # Text height

            # Smooth gradient: Red (low) → Green (high)
            level_norm = max(0.0, min( level/ 100.0, 1.0))
            marker.color.r = 1.0 - level_norm
            marker.color.g = level_norm
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.text = f"{robot_name}: {int(level)}%"
            marker_array.markers.append(marker)


        self.publisher.publish(marker_array)
        return py_trees.common.Status.SUCCESS

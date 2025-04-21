import rospy
import py_trees
import py_trees_ros.trees
from coverage_application.behaviours import HasPosesToCover, MockTask, HasBattery, MoveToFillBattery, MoveToNextPoseClient, PublishCoverageMarkers

class CoverageTree(py_trees_ros.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("[CoverageTree] Initializing tree...")
        root = self.create_tree_root()
        super(CoverageTree, self).__init__(root)

    def create_robot_branch(self, robot_id):
        # Create a branch for robot
        move_to_pose = MoveToNextPoseClient(
            name="MoveToNextPoseClient",
            action_namespace=f"/robot{robot_id+1}/move_base",
            robot_id=robot_id
        )
        has_battery = HasBattery(
            name="HasBattery",
            robot_id=robot_id,
            low_battery_threshold=20.0
        )
        move_to_fill_battery = MoveToFillBattery(
            name="MoveToFillBattery",
            action_namespace=f"/robot{robot_id+1}/move_base",
            robot_id=robot_id
        )

        coverage_explore = py_trees.composites.Sequence(
            "CoverageSequence",
            children=[
                move_to_pose,
                #MockTask(name="MockTask", duration=5.0),
                #py_trees.behaviours.Failure(),
                #CheckAllGoals(name="CheckAllGoals"),
                #py_trees.behaviours.Running(),
                #py_trees.behaviours.Success(name="MapCovered"),
            ]
        )

        has_battery_selector = py_trees.composites.Selector(
            "HasBatterySelector",
            children=[
                has_battery,
                move_to_fill_battery,
            ]
        )

        robot_has_battery_sequence = py_trees.composites.Sequence(
            f"Robot{robot_id}Sequence",
            children=[
                has_battery_selector,
                coverage_explore,
            ]
        )
        
        has_poses_to_cover = py_trees.composites.Selector(
            "HasPosesToCoverSelector",
            children=[
                HasPosesToCover(name="HasPosesToCover"),
                py_trees.behaviours.Running(),
            ]
        )  

        
        robot_has_battery_and_poses_sequence = py_trees.composites.Sequence(
            f"Robot{robot_id}AndPosesSequence",
            children=[
                has_poses_to_cover,
                robot_has_battery_sequence,
            ]
        )

        return robot_has_battery_and_poses_sequence


    def create_tree_root(self):


        robot1 = self.create_robot_branch(robot_id=0)
        robot2 = self.create_robot_branch(robot_id=1)


        robots = py_trees.composites.Parallel(
            "CoverageParallel",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            children=[
                robot1,
                robot2,
            ]
        )

        root = py_trees.composites.Parallel(
            "CoverageSelector",
            children=[
                PublishCoverageMarkers(name="PublishCoverageMarkers"),
                robots,
            ]
        )
        
        return root

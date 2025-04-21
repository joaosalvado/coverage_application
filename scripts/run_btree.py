#!/usr/bin/env python3
import rospy
from coverage_application.tree import CoverageTree

def main():
    rospy.init_node("coverage_tree")

    tree = CoverageTree()
    tree.setup(timeout=15)

    rospy.loginfo("[run_tree] Starting tree tick loop...")

    tree.tick_tock(500)
    rospy.loginfo("[run_tree] Shutting down...")
    
if __name__ == '__main__':
    main()

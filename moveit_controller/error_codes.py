#!/usr/bin/env python3

from moveit_msgs.msg import MoveItErrorCodes

error_code_dict = {
    MoveItErrorCodes.SUCCESS: "Success",
    MoveItErrorCodes.FAILURE: "Failure",
    MoveItErrorCodes.PLANNING_FAILED: "Planning failed",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "Motion plan invalidated by environment change",
    MoveItErrorCodes.CONTROL_FAILED: "Control failed",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "Unable to acquire sensor data",
    MoveItErrorCodes.TIMED_OUT: "Timed out",
    MoveItErrorCodes.PREEMPTED: "Preempted",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "Start state in collision",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "Start state violates path constraints",
    MoveItErrorCodes.GOAL_IN_COLLISION: "Goal in collision",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "Goal violates path constraints",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "Goal constraints violated",
    MoveItErrorCodes.INVALID_GROUP_NAME: "Invalid group name",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "Invalid goal constraints",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "Invalid robot state",
    MoveItErrorCodes.INVALID_LINK_NAME: "Invalid link name",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "Invalid object name",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "Frame transform failure",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "Collision checking unavailable",
    MoveItErrorCodes.ROBOT_STATE_STALE: "Robot state stale",
    MoveItErrorCodes.SENSOR_INFO_STALE: "Sensor info stale",
    MoveItErrorCodes.COMMUNICATION_FAILURE: "Communication failure",
    MoveItErrorCodes.NO_IK_SOLUTION: "No IK solution",
}
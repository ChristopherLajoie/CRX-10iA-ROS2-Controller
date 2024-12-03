#!/usr/bin/env python3

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
)

def build_goal_msg(joint_positions, planner_id='', pipeline_id='ompl'):
    goal_msg = MoveGroup.Goal()

    goal_msg.request = MotionPlanRequest()
    goal_msg.request.workspace_parameters = WorkspaceParameters()
    goal_msg.request.start_state.is_diff = True
    goal_msg.request.group_name = 'arm'
    goal_msg.request.allowed_planning_time = 30.0
    goal_msg.request.planner_id = planner_id
    goal_msg.request.pipeline_id = pipeline_id

    constraints = Constraints()

    joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

    for name, position in zip(joint_names, joint_positions):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = name
        joint_constraint.position = position
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

    goal_msg.request.goal_constraints.append(constraints)
    goal_msg.planning_options = PlanningOptions()
    goal_msg.planning_options.plan_only = True

    return goal_msg

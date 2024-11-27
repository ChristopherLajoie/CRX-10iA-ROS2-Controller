#!/usr/bin/env python3

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
)

def build_goal_msg(
    joint_positions,
    group_name='arm',
    planner_id='RRTConnectkConfigDefault',
    num_planning_attempts=10,
    allowed_planning_time=15.0,
    max_velocity_scaling_factor=1.0,
    max_acceleration_scaling_factor=1.0,
    plan_only=True
):
    goal_msg = MoveGroup.Goal()

    goal_msg.request = MotionPlanRequest()
    goal_msg.request.workspace_parameters = WorkspaceParameters()
    goal_msg.request.start_state.is_diff = True
    goal_msg.request.planner_id = planner_id
    goal_msg.request.group_name = group_name
    goal_msg.request.num_planning_attempts = num_planning_attempts
    goal_msg.request.allowed_planning_time = allowed_planning_time
    goal_msg.request.max_velocity_scaling_factor = max_velocity_scaling_factor
    goal_msg.request.max_acceleration_scaling_factor = max_acceleration_scaling_factor

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
    goal_msg.planning_options.plan_only = plan_only

    return goal_msg
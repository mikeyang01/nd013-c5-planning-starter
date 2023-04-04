/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
/**
 * @file behavior_planner_FSM.cpp
 **/
#include "behavior_planner_FSM.h"
using namespace std::chrono;

State BehaviorPlannerFSM::get_closest_waypoint_goal(
  const State & ego_state,
  const SharedPtr < cc::Map > & map,
  const float & lookahead_distance, bool & is_goal_junction) {
  

  // Nearest waypoint on the center of a Driving Lane.
  auto waypoint_0 = map -> GetWaypoint(ego_state.location);

  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
    State waypoint;
    auto wp_transform = waypoint_0 -> GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
    return waypoint;
  }
  
  /* 
  Waypoints at a lookahead distance
  NOTE: "GetNext(d)" creates a list of waypoints at an approximate distance "d" in the direction of the lane. 
  The list contains one waypoint for each deviation possible.
  NOTE 2: GetNextUntilLaneEnd(d) returns a list of waypoints a distance "d" apart. 
  The list goes from the current waypoint to the end of its lane.
  获取车道上某个前瞻距离内的路径点。
  
  GetNext(d)函数会返回一个列表，其中包含了在车道方向上距离当前位置大约为d的路径点。
  这个列表包含了每个可能的偏差方向上的一个路径点。这个函数可以帮助车辆预测未来的路径，并做出相应的决策。
  
  GetNextUntilLaneEnd(d)函数则会返回一个列表，其中包含了从当前路径点到车道末端距离为d的路径点。这个函数可以帮助车辆规划行驶到车道末端的路径。
  */
  auto lookahead_waypoints = waypoint_0 -> GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    State waypoint;
    return waypoint;
  }

  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

  is_goal_junction = waypoint_0 -> IsJunction();

  auto cur_junction_id = waypoint_0 -> GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      // LOG(INFO) << "BP - Last wp is in same junction as ego. Junction ID: "
      is_goal_junction = false;
    } else {
      // LOG(INFO) << "BP - Last wp is in different junction than ego. Junction
      _prev_junction_id = cur_junction_id;
    }
  }

  State waypoint;
  auto wp_transform = waypoint_0 -> GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
  return waypoint;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State & ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  /* 
  TODO-Lookahead: 
  One way to find a reasonable lookahead distance is to 
  find the distance you will need to come to a stop while traveling at speed V and using a comfortable deceleration.
  */
  VelocityProfileGenerator vpg;
  auto look_ahead_distance = vpg.calc_distance(velocity_mag, 0, _max_accel); // <- Fix This

  //使用std::min()和std::max()函数对前瞻距离进行限制。这个限制可以确保前瞻距离在一定的范围内，以避免车辆做出不安全的速度规划。
  look_ahead_distance = std::min(std::max(look_ahead_distance, _lookahead_distance_min), _lookahead_distance_max);

  return look_ahead_distance;
}

State BehaviorPlannerFSM::get_goal(const State & ego_state, SharedPtr < cc::Map > map) {
  /* 
  Get look-ahead distance based on Ego speed  
  ego_speed是车辆的速度，
  lookahead_time是我们希望车辆能够提前看到的时间。
  例如，如果我们希望车辆能够提前2秒看到前方的道路情况，那么我们可以将lookahead_time设置为2秒。
  */
  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  bool is_goal_in_junction {
    false
  };

  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
    is_goal_in_junction);

  // LOG(INFO) << "Is the FINAL goal on a junction: " << is_goal_in_junction;
  string tl_state = "none";
  State goal =
    state_transition(ego_state, goal_wp, is_goal_in_junction, tl_state);

  return goal;
}

State BehaviorPlannerFSM::state_transition(
  const State & ego_state, 
  State goal,
  bool & is_goal_in_junction,
  string tl_state) {
  // Check with the Behavior Planner to see what we are going to do and where our next goal is
  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

  if (_active_maneuver == FOLLOW_LANE) {
    // LOG(INFO) << "BP- IN FOLLOW_LANE STATE";
    if (is_goal_in_junction) {
      // LOG(INFO) << "BP - goal in junction";

      _active_maneuver = DECEL_TO_STOP;
      // LOG(INFO) << "BP - changing to DECEL_TO_STOP";
      // Let's backup a "buffer" distance behind the "STOP" point      
      /*
      TODO:
      goal behind the stopping point: 
      put the goal behind the stopping point (i.e the actual goal location) by "_stop_line_buffer". 
      HINTS:
      remember that we need to go back in the opposite direction of the goal/road, 
      i.e you should use: ang = goal.rotation.yaw + M_PI and then use cosine and sine to get x and y
      */
      auto ang = goal.rotation.yaw + M_PI;
      //更新目标位置的x坐标
      goal.location.x += std::cos(ang); // <- Fix This
      goal.location.y += std::sin(ang); // <- Fix This

      // LOG(INFO) << "BP- new STOP goal at: " << goal.location.x << ", " << goal.location.y;

      // TODO-goal speed at stopping point: What should be the goal speed
      // 停止当然是0
      goal.velocity.x = 0; // <- Fix This
      goal.velocity.y = 0; // <- Fix This
      goal.velocity.z = 0; // <- Fix This

    } else {
      /* 
      TODO:
      goal speed in nominal state: 
      What should be the goal speed now that we know we are in nominal state and we can continue freely?
      Remember that the speed is a vector
      HINT: _speed_limit * std::sin/cos (goal.rotation.yaw);
      为了计算x和y分量，我们可以使用三角函数
      */
      goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw); // <- Fix This
      goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw); // <- Fix This
      goal.velocity.z = 0;
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {
    // LOG(INFO) << "BP- IN DECEL_TO_STOP STATE";
    /*
    TODO:
    maintain the same goal when in DECEL_TO_STOP state: 
    Make sure the new goal is the same as the previous goal (_goal). 
    That way we keep/maintain the goal at the stop line.    
    DECEL_TO_STOP状态是指车辆正在减速以到达停车线。
    在这个状态下，我们需要确保车辆的目标点与之前的目标点相同，以便车辆在停车线上保持目标点。
    */
    goal = _goal; // <- Fix This

    /* 
    TODO: 
    It turns out that when we teleport, the car is always at speed zero. 
    In this the case, as soon as we enter the DECEL_TO_STOP state,
    the condition that we are <= _stop_threshold_speed is ALWAYS true and we move straight to "STOPPED" state. 
    To solve this issue (since we don't have a motion controller yet), 
    you should use "distance" instead of speed. 
    Make sure the distance to the stopping point is <= P_STOP_THRESHOLD_DISTANCE. 
    Uncomment the line used to calculate the distance
    */
    auto distance_to_stop_sign = utils::magnitude(goal.location - ego_state.location);

    if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) {// -> Fix this
      /* 
      TODO-move to STOPPED state: 
      Now that we know we are close or at the stopping point we should change state to "STOPPED"
      */
      _active_maneuver = STOPPED; // <- Fix This
      _start_stop_time = std::chrono::high_resolution_clock::now();
      // LOG(INFO) << "BP - changing to STOPPED";
    }

  } else if (_active_maneuver == STOPPED) {
    /*
    TODO:
    maintain the same goal when in STOPPED state: 
    Make sure the new goal is the same as the previous goal.
    That way we keep/maintain the goal at the stop line. 
    */
    goal = _goal; // Keep previous goal. Stay where you are. // <- Fix This

    long long stopped_secs = duration_cast <seconds> (high_resolution_clock::now() - _start_stop_time)
      .count();
    // LOG(INFO) << "BP- Stopped for " << stopped_secs << " secs";

    /*
    如果车辆已经停止了一段时间（stopped_secs >= _req_stop_time），
    并且交通灯状态不是红灯（tl_state.compare("Red") != 0），
    那么车辆需要执行一些操作来继续行驶。*/
    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) {
      /*TODO:
      move to FOLLOW_LANE state: What state do we want to move to, when we are "done" at the STOPPED state?
      */
      _active_maneuver = FOLLOW_LANE; // <- Fix This
      // LOG(INFO) << "BP - changing to FOLLOW_LANE";
    }
  }
  _goal = goal;
  return goal;
}
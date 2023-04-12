/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
/**
 * @file behavior_planner_FSM.cpp
 **/
#include "behavior_planner_FSM.h"
#include "velocity_profile_generator.h"

using namespace std;
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
  Waypoints at a lookahead distance: 获取车道上某个前瞻距离内的路径点。
  GetNext(d)
  	函数会返回一个列表，其中包含了在车道方向上距离当前位置大约为d的路径点。
  	这个列表包含了每个可能的偏差方向上的一个路径点。这个函数可以帮助车辆预测未来的路径，并做出相应的决策。  
  GetNextUntilLaneEnd(d)
  	函数则会返回一个列表，其中包含了从当前路径点到车道末端距离为d的路径点。
  	这个函数可以帮助车辆规划行驶到车道末端的路径。
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
  ------ TODO-Lookahead: ------ 
  One way to find a reasonable lookahead distance is to 
  find the distance you will need to come to a stop while traveling at speed V and using a comfortable deceleration.
  前瞻距离
  	是指车辆在行驶过程中所能看到的最远的距离。
  	在自动驾驶汽车中，前瞻距离非常重要，因为它可以帮助车辆更好地规划路径和避免碰撞。
    找到一个合理的前瞻距离的方法之一是计算在以速度V行驶并使用舒适的减速度时，需要停下来的距离。
    这个距离就是合理的前瞻距离。
    这个距离可以根据车辆的速度和减速度来计算，从而确保车辆在行驶过程中有足够的时间来做出反应并避免碰撞。
    例如，车辆以每小时50公里的速度行驶，并且使用一个舒适的减速度，那么需要的停车距离可能是30米。30米是车辆的前瞻距离。
  */
  VelocityProfileGenerator vpg;
  auto look_ahead_distance = vpg.calc_distance(velocity_mag, 0, _max_accel); // <- Fix This

  //使用std::min()和std::max()函数对前瞻距离进行限制。
  //这个限制可以确保前瞻距离在一定的范围内，以避免车辆做出不安全的速度规划。
  look_ahead_distance = min(max(look_ahead_distance, _lookahead_distance_min),_lookahead_distance_max);

  return look_ahead_distance;
}


/*函数用途: 通过获取目标状态，帮助自动驾驶汽车规划出下一步的行动
	接受两个参数：一个是车辆的状态ego_state，另一个是地图map。
	在这个函数中，首先会根据车辆的状态和地图信息，计算出车辆当前所在的车道。
    然后，根据车辆所在的车道，以及车辆的速度和加速度等信息，计算出车辆的目标状态。*/
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


/*
行为规划器
	可以根据车辆的状态和环境信息，规划出车辆的行为和路径。
	在行为规划器中，状态机是一个常用的工具，用来描述车辆的状态和行为之间的关系。
函数作用: 根据当前状态和目标状态, 计算出车辆需要采取的行动。
	在这个函数中，我们可以看到它接受两个参数：一个是当前状态current_state，另一个是目标状态target_state。
    它会根据当前状态和目标状态之间的差异，决定车辆需要采取的行动，例如加速、减速、转弯等。
	例如，如果当前状态是“跟随车道”，目标状态是“减速停车”，那么这个函数会计算出车辆需要减速并停车的行动。
    如果当前状态是“减速停车”，目标状态是“保持停车”，那么这个函数会计算出车辆需要保持停车的行动。*/
State BehaviorPlannerFSM::state_transition(
  const State & ego_state, 
  State goal,
  bool & is_goal_in_junction,
  string tl_state) {
  // Check with the Behavior Planner to see what we are going to do and where our next goal is
  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;

  /*如果车辆处于“Follow_lane”状态，那么就需要根据目标点是否在路口内来判断是否需要进行减速停车操作。*/
  if (_active_maneuver == FOLLOW_LANE) {

    /*如果目标点在路口内，那么就需要进行减速停车操作。
    车辆接近路口时，能够检测到路口，并在路口处进行减速停车，等待通过路口的信号。如果目标点不在路口内，车辆可以原速度行驶*/
    if (is_goal_in_junction) {
      _active_maneuver = DECEL_TO_STOP;
      
      /*------ TODO: ------ 
      goal behind the stopping point: 
      put the goal behind the stopping point (i.e the actual goal location) by "_stop_line_buffer". 
      HINTS:
      remember that we need to go back in the opposite direction of the goal/road, 
      i.e you should use: ang = goal.rotation.yaw + M_PI and then use cosine and sine to get x and y
      */
      auto ang = goal.rotation.yaw + M_PI;
      //更新目标位置的x坐标
      goal.location.x += cos(ang); // <- Fix This
      goal.location.y += sin(ang); // <- Fix This

      /*------ TODO ------ 
      goal speed at stopping point: What should be the goal speed: 停止当然是0
      */
      goal.velocity.x = 0; // <- Fix This
      goal.velocity.y = 0; // <- Fix This
      goal.velocity.z = 0; // <- Fix This
      
    } else {
      /* 
      ------ TODO: ------ 
      What should be the goal speed now that we know we are in nominal state and we can continue?
      Remember that the speed is a vector
      HINT: _speed_limit * std::sin/cos (goal.rotation.yaw);
      为了计算x和y分量，我们可以使用三角函数
      */
      goal.velocity.x = _speed_limit * cos(goal.rotation.yaw); // <- Fix This
      goal.velocity.y = _speed_limit * sin(goal.rotation.yaw); // <- Fix This
      goal.velocity.z = 0;
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {
    /* ------ TODO: ------ 
    maintain the same goal when in DECEL_TO_STOP state: 
    Make sure the new goal is the same as the previous goal (_goal). 
    That way we keep/maintain the goal at the stop line.    
    DECEL_TO_STOP状态是指车辆正在减速以到达停车线。
    在这个状态下，我们需要确保车辆的目标点与之前的目标点相同，以便车辆在停车线上保持目标点。*/
    goal = _goal; // <- Fix This

    /* ------ TODO: ------ 
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
      ------ TODO: move to STOPPED state: ------ 
      Now that we know we are close or at the stopping point we should change state to "STOPPED"
      */
      _active_maneuver = STOPPED; // <- Fix This
      _start_stop_time = high_resolution_clock::now();
      // LOG(INFO) << "BP - changing to STOPPED";
    }

  } else if (_active_maneuver == STOPPED) {
    /*------ TODO: ------ 
    maintain the same goal when in STOPPED state: 
    Make sure the new goal is the same as the previous goal.
    That way we keep/maintain the goal at the stop line. 
    车辆处于“STOPPED”状态时，维持车辆的目标点不变，并在一定时间后判断交通灯状态，从而决定车辆下一步的行动。
	在这个状态下，我们需要判断车辆已经停止的时间是否超过了要求的停止时间，并且需要判断交通灯的状态是否为红灯。
    如果车辆已经停止了一定时间，并且交通灯状态不是红灯，那么车辆需要执行一些操作来继续行驶。
    */
    goal = _goal; // Keep previous goal. Stay where you are. // <- Fix This

    long long stopped_secs = duration_cast <seconds> (high_resolution_clock::now() - _start_stop_time)
      .count();
    
    /*如果车辆已经停止了一段时间（stopped_secs >= _req_stop_time），
    并且交通灯状态不是红灯（tl_state.compare("Red") != 0），
    那么车辆需要执行一些操作来继续行驶。*/
    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) {
      /*------ TODO: ------ 
      move to FOLLOW_LANE state: 
      What state do we want to move to, when we are "done" at the STOPPED state?
      */
      _active_maneuver = FOLLOW_LANE; // <- Fix This          
    }
  }
  _goal = goal;
  return goal;
}
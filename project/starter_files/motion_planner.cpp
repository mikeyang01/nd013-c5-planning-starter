/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
#include "motion_planner.h"

MotionPlanner::~MotionPlanner() {}

State MotionPlanner::get_goal_state_in_ego_frame(const State& ego_state,
                                                 const State& goal_state) {
  // Let's start by making a copy of the goal state (global reference frame)
  auto goal_state_ego_frame = goal_state;

  /*
  Translate so the ego state is at the origin in the new frame.
  This is done by subtracting the ego_state from the goal_ego_.
  这里的目标是将自动驾驶汽车的状态转换到一个新的参考系中，使得自动驾驶汽车的状态处于参考系的原点。
  这个新的参考系被称为“ego frame”，也就是自我参考系。
  将自动驾驶汽车的状态从全局参考系转换到自我参考系。这是因为在自我参考系中，自动驾驶汽车的状态更容易被描述和控制。

  接下来的三行代码是将目标状态从全局参考系转换到自我参考系。
  这样做的结果是，目标状态现在是相对于自动驾驶汽车的状态的偏移量，而不是相对于全局参考系的绝对位置。
  */
  goal_state_ego_frame.location.x -= ego_state.location.x;
  goal_state_ego_frame.location.y -= ego_state.location.y;
  goal_state_ego_frame.location.z -= ego_state.location.z;

  /*
  Rotate such that the ego state has zero heading/yaw in the new frame.
  We are rotating by -ego_state "yaw" to ensure the ego vehicle's
  current yaw corresponds to theta = 0 in the new local frame.

  Recall that the general rotation matrix around the Z axix is:
     [cos(theta) -sin(theta)
     sin(theta)  cos(theta)]
  */
  auto theta_rad = -ego_state.rotation.yaw;
  auto cos_theta = std::cos(theta_rad);
  auto sin_theta = std::sin(theta_rad);

  goal_state_ego_frame.location.x =
      cos_theta * goal_state_ego_frame.location.x -
      sin_theta * goal_state_ego_frame.location.y;
  goal_state_ego_frame.location.y =
      sin_theta * goal_state_ego_frame.location.x +
      cos_theta * goal_state_ego_frame.location.y;

  // Compute the goal yaw in the local frame by subtracting off the current ego
  // yaw from the goal waypoint heading/yaw.
  goal_state_ego_frame.rotation.yaw += theta_rad;

  // Ego speed is the same in both coordinates the Z coordinate does not get
  // affected by the rotation. Let's make sure the yaw is within [-180, 180] or
  // [-pi, pi] so the optimizer works.
  goal_state_ego_frame.rotation.yaw = 
    utils::keep_angle_range_rad(goal_state_ego_frame.rotation.yaw, -M_PI, M_PI);
  // if (goal_state_ego_frame.rotation.yaw < -M_PI) {
  //   goal_state_ego_frame.rotation.yaw += (2 * M_PI);
  // } else if (goal_state_ego_frame.rotation.yaw > M_PI) {
  //   goal_state_ego_frame.rotation.yaw -= (2 * M_PI);
  // }

  return goal_state_ego_frame;
}

std::vector<State> MotionPlanner::generate_offset_goals_ego_frame(
    const State& ego_state, const State& goal_state) {
  // Let's transform the "main" goal (goal state) into ego reference frame
  auto goal_state_ego_frame =
      get_goal_state_in_ego_frame(ego_state, goal_state);

  return generate_offset_goals(goal_state_ego_frame);
}

std::vector<State> MotionPlanner::generate_offset_goals_global_frame(
    const State& goal_state) {
  return generate_offset_goals(goal_state);
}

std::vector<State> MotionPlanner::generate_offset_goals(
    const State& goal_state) {
  /*
  Now we need to generate "_num_paths" goals offset from the center goal at a
  distance "_goal_offset".
  这个函数的作用是生成一些偏移目标状态，以便我们可以在路径规划中考虑不同的车道位置。
  例如，如果我们想让车辆保持在中央车道，我们可以生成一些偏移目标状态，将车辆的目标位置向左或向右偏移一些距离。
  */
  std::vector<State> goals_offset;

  /*
  the goals will be aligned on a perpendicular line to the heading of the main goal. 
  To get a perpendicular angle, just add 90 (or PI/2) to the main goal heading.
  
  TODO:
  Perpendicular direction: ADD pi/2 to the goal yaw (goal_state.rotation.yaw)
  在这段代码中，我们将目标状态的偏航角加上90度，以便将其旋转90度。
  这个旋转是为了使自动驾驶汽车朝着目标状态的侧面移动，而不是直接向目标状态移动。
  例如，假设自动驾驶汽车的目标状态是一个停在路边的汽车。
  如果我们直接向目标状态移动，我们可能会撞到停在路边的汽车。
  但是，如果我们将目标状态旋转90度，我们就可以朝着目标状态的侧面移动，而不会撞到停在路边的汽车。
  */
  auto yaw_plus_90 = goal_state.rotation.yaw + M_PI / 2;  // <- Fix This

  for (int i = 0; i < _num_paths; ++i) {
    auto goal_offset = goal_state;
    float offset = (i - (int)(_num_paths / 2)) * _goal_offset;

    /*
    TODO-offset goal location:
    calculate the x and y position of the offset goals using "offset" (calculated above) 
    and knowing that the goals should lie on a perpendicular line to the direction (yaw) of the main goal. 
    You calculated this direction above (yaw_plus_90). 
    HINT: use std::cos(yaw_plus_90) and std::sin(yaw_plus_90)
    */
    goal_offset.location.x += offset * std::cos(yaw_plus_90);  // <- Fix This
    goal_offset.location.y += offset * std::sin(yaw_plus_90);  // <- Fix This

    if (valid_goal(goal_state, goal_offset)) {
      goals_offset.push_back(goal_offset);
    }
  }
  return goals_offset;
}

bool MotionPlanner::valid_goal(const State& main_goal,
                               const State& offset_goal) {
  auto max_offset = ((int)(_num_paths / 2) + 1) * _goal_offset;
  // LOG(INFO) << "max offset: " << max_offset;
  auto dist = utils::magnitude(main_goal.location - offset_goal.location);
  // LOG(INFO) << "distance from main goal to offset goal: " << dist;
  return dist < max_offset;
}

/*
当我们在自动驾驶汽车的运动规划和决策制定中使用螺旋路径时，
我们需要选择最佳的螺旋路径，以便自动驾驶汽车可以安全地到达目标状态。
*/
std::vector<int> MotionPlanner::get_best_spiral_idx(
    const std::vector<std::vector<PathPoint>>& spirals,
    const std::vector<State>& obstacles, const State& goal_state) {
  // LOG(INFO) << "Num Spirals: " << spirals.size();
  double best_cost = DBL_MAX;
  std::vector<int> collisions;
  int best_spiral_idx = -1;
  for (size_t i = 0; i < spirals.size(); ++i) {
    double cost = calculate_cost(spirals[i], obstacles, goal_state);

    if (cost < best_cost) {
      best_cost = cost;
      best_spiral_idx = i;
    }
    if (cost > DBL_MAX) {
      collisions.push_back(i);
    }
  }
  if (best_spiral_idx != -1) {
    collisions.push_back(best_spiral_idx);
    return collisions;
  }
  std::vector<int> noResults;
  return noResults;
}

/*在这段代码中，我们需要将自动驾驶汽车的运动轨迹从自我参考系转换到全局参考系。*/
std::vector<std::vector<PathPoint>>
MotionPlanner::transform_spirals_to_global_frame(
    const std::vector<std::vector<PathPoint>>& spirals,
    const State& ego_state) {
  std::vector<std::vector<PathPoint>> transformed_spirals;
  for (auto spiral : spirals) {
    std::vector<PathPoint> transformed_single_spiral;
    for (auto path_point : spiral) {
      PathPoint new_path_point;
      new_path_point.x = ego_state.location.x +
                         path_point.x * std::cos(ego_state.rotation.yaw) -
                         path_point.y * std::sin(ego_state.rotation.yaw);
      new_path_point.y = ego_state.location.y +
                         path_point.x * std::sin(ego_state.rotation.yaw) +
                         path_point.y * std::cos(ego_state.rotation.yaw);
      new_path_point.theta = path_point.theta + ego_state.rotation.yaw;

      transformed_single_spiral.emplace_back(new_path_point);
    }
    transformed_spirals.emplace_back(transformed_single_spiral);
  }
  return transformed_spirals;
}

/*这个函数的目的是生成一组螺旋路径，以便自动驾驶汽车可以沿着这些路径行驶，从而到达目标状态。*/
std::vector<std::vector<PathPoint>> MotionPlanner::generate_spirals(
    const State& ego_state, const std::vector<State>& goals) {
  // Since we are on Ego Frame, the start point is always at 0, 0, 0,
  PathPoint start;
  start.x = ego_state.location.x;
  start.y = ego_state.location.y;
  start.z = ego_state.location.z;
  start.theta = ego_state.rotation.yaw;
  start.kappa = 0.0;
  start.s = 0.0;
  start.dkappa = 0.0;
  start.ddkappa = 0.0;

  std::vector<std::vector<PathPoint>> spirals;
  for (auto goal : goals) {
    PathPoint end;
    end.x = goal.location.x;
    end.y = goal.location.y;
    end.z = goal.location.z;
    end.theta = goal.rotation.yaw;
    end.kappa = 0.0;
    end.s = std::sqrt((end.x * end.x) + (end.y * end.y));
    end.dkappa = 0.0;
    end.ddkappa = 0.0;

    if (_cubic_spiral.GenerateSpiral(start, end)) {
      std::vector<PathPoint>* spiral = new std::vector<PathPoint>;
      auto ok = _cubic_spiral.GetSampledSpiral(P_NUM_POINTS_IN_SPIRAL, spiral);
      if (ok && valid_spiral(*spiral, goal)) {
        // LOG(INFO) << "Spiral Valid ";
        spirals.push_back(*spiral);
      } else {
        // LOG(INFO) << "Spiral Invalid ";
      }
    } else {
      // LOG(INFO) << "Spiral Generation FAILED! ";
    }
  }
  return spirals;
}

/*
当我们进行自动驾驶汽车的运动规划时，我们需要确保生成的路径是有效的，可以安全地导航自动驾驶汽车到达目标位置。
这个函数的目的是检查生成的螺旋路径是否有效，以便我们可以在自动驾驶汽车上使用它。
*/
bool MotionPlanner::valid_spiral(const std::vector<PathPoint>& spiral,
                                 const State& offset_goal) {
  auto n = spiral.size();
  auto delta_x = (offset_goal.location.x - spiral[n - 1].x);
  auto delta_y = (offset_goal.location.y - spiral[n - 1].y);
  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
  // auto dist = utils::magnitude(spiral[spiral.size() - 1].location -
  //                              offset_goal.location);
  // LOG(INFO) << "Distance from Spiral end to offset_goal: " << dist;
  return (dist < 0.1);
}

/*这个函数的目的是计算一条螺旋路径的代价，以便确定哪条路径是最优的。
这个代价是根据路径上的位置、速度和加速度等信息以及障碍物的位置和目标状态来计算的。*/
float MotionPlanner::calculate_cost(const std::vector<PathPoint>& spiral,
                                    const std::vector<State>& obstacles,
                                    const State& goal) {
  // LOG(INFO) << "Starting spiral cost calc";
  // Initialize cost to 0.0
  float cost = 0.0;
  cost += cf::collision_circles_cost_spiral(spiral, obstacles);

  cost += cf::close_to_main_goal_cost_spiral(spiral, goal);

  // LOG(INFO) << "Path Cost: " << cost;
  return cost;
}

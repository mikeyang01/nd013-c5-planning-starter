/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
/**
 * @file cost_functions.cpp
 **/
#include "cost_functions.h"

using namespace std;

//C++基础:命名空间可以帮助我们将代码组织成逻辑上相关的部分，以便更好地管理和维护代码。

/*    
我们希望找到一条轨迹，使得该轨迹与目标尽可能接近。
    	为了实现这一点，例如最小二乘法或梯度下降法, 使得轨迹上的坐标和导数与目标尽可能接近。
        在这个过程中，我们需要定义一个代价函数，它可以衡量轨迹与目标之间的差异。
        代价函数可以是任何函数，只要它能够衡量轨迹与目标之间的差异即可。*/
namespace cost_functions {
  
  double diff_cost(vector < double > coeff, double duration, array < double, 3 > goals,
    array < float, 3 > sigma, double cost_weight) {
    /*
    Penalizes trajectories whose coordinate(and derivatives) differ from the goal.
    */
    
    double cost = 0.0;
    vector < double > evals = evaluate_f_and_N_derivatives(coeff, duration, 2);

    for (size_t i = 0; i < evals.size(); i++) {
      double diff = fabs(evals[i] - goals[i]);
      cost += logistic(diff / sigma[i]);
    }
  
    return cost_weight * cost;
  }

  // 计算螺旋路径与障碍物之间的碰撞代价。
  double collision_circles_cost_spiral(const std::vector < PathPoint > & spiral,
    const std::vector < State > & obstacles) {
    bool collision {
      false
    };
    auto n_circles = CIRCLE_OFFSETS.size();

    for (auto wp: spiral) {
      if (collision) {//碰撞发生
        break;
      }
      
      double cur_x = wp.x;
      double cur_y = wp.y;
      double cur_yaw = wp.theta; // This is already in rad.

      for (size_t c = 0; c < n_circles && !collision; ++c) {
        /*
        ------ TODO: ------
        计算圆形障碍物的中心坐标。
        	cur_x和cur_y是车辆当前的x和y坐标，cur_yaw是车辆当前的航向角。
        	CIRCLE_OFFSETS[c]是一个常量数组，包含了每个圆形障碍物相对于车辆中心的偏移量
        */
        auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * cos(cur_yaw); // <- fix 
        auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * sin(cur_yaw); // <- fix

        for (auto obst: obstacles) {
          if (collision) {
            break;
          }
          
          auto actor_yaw = obst.rotation.yaw;
          for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
            auto actor_center_x =
              obst.location.x + CIRCLE_OFFSETS[c2] * std::cos(actor_yaw);
            auto actor_center_y =
              obst.location.y + CIRCLE_OFFSETS[c2] * std::sin(actor_yaw);
            /*
            ------ TODO: ------
            Distance from circles to obstacles/actor:
            	勾股定理*/
            double dist = sqrt(
              pow(circle_center_x - actor_center_x, 2) +
              pow(circle_center_y - actor_center_y, 2)); // <- fix

            /*如果圆形障碍物与车辆之间的距离小于它们的半径之和，那么就意味着车辆与障碍物发生了碰撞。
            在这种情况下，我们将设置一个collision标志，以便在后续的代码中处理碰撞情况。*/
            collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
          }
        }
      }
    }
    return (collision) ? COLLISION : 0.0;
  }

  
  /*
  这个函数中，我们使用螺旋路径来生成一条路径，并计算它与主目标之间的距离成本。
  	螺旋路径是一种特殊的路径，它可以帮助我们生成平滑的路径，并且可以避免车辆在行驶过程中出现急转弯或急刹车等不舒适的情况。
  	在自动驾驶车辆的路径规划中，我们需要选择一条最佳路径，使得车辆能够安全地行驶到目的地。
  这个函数会使用螺旋路径的最后一个点来计算路径与主目标之间的距离成本。
  	如果螺旋路径的最后一个点距离车道中心线更近，并且没有碰撞风险，那么这条路径的成本就会更低，也就更有可能被选择为最佳路径
  */
  double close_to_main_goal_cost_spiral(const std::vector < PathPoint > & spiral,
    State main_goal) {
    auto n = spiral.size();

    /*
    ------ TODO-distance between last point on spiral and main goal:------
    How do we calculate the distance between the last point on the spiral (spiral[n-1]) and the main goal (main_goal.location). 
    	Use spiral[n - 1].x, spiral[n - 1].y and spiral[n - 1].z. 
    	Use main_goal.location.x, main_goal.location.y and main_goal.location.z 
    	Ex: main_goal.location.x - spiral[n - 1].x     
    delta_x：表示车辆当前位置与目标位置之间在 x 轴上的距离。
    main_goal.location.x：表示目标位置在 x 轴上的位置。
    spiral[n - 1].x：表示车辆当前位置在 x 轴上的位置。
    */
    auto delta_x = main_goal.location.x - spiral[n - 1].x; // <- fix
    auto delta_y = main_goal.location.y - spiral[n - 1].y; // <- fix
    auto delta_z = main_goal.location.z - spiral[n - 1].z; // <- fix

    auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y) +
      (delta_z * delta_z));

    auto cost = logistic(dist);
    // LOG(INFO) << "distance to main goal: " << dist;
    // LOG(INFO) << "cost (log): " << cost;
    return cost;
  }
}
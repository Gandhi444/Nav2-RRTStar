/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include <random>
#include "nav2_util/node_utils.hpp"

#include "nav2_rrtstar_planner/rrtstar_planner.hpp"

namespace nav2_rrtstar_planner
{

  void rrtstar::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    srand(static_cast<unsigned>(time(0)));
  }

  void rrtstar::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void rrtstar::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void rrtstar::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }
  int rrtstar::costBeetweanPoints(double x1, double y1, double x2, double y2, bool &encoutered_obstacle)
  {
    int total_number_of_loop = std::hypot(
                                   x1 - x2, y1 - y2) /
                               interpolation_resolution_;
    double x_increment = (x1 - x2) / total_number_of_loop;
    double y_increment = (y1 - y2) / total_number_of_loop;
    double TotalCost = 0;
    unsigned int prev_index = UINT_MAX;
    for (int j = 0; j < total_number_of_loop; ++j)
    {
      double position_x = x2 + x_increment * j;
      double position_y = y2 + y_increment * j;
      unsigned int new_index = costmap_->getIndex(position_x / mPerCellX, position_y / mPerCellY);
      if (new_index != prev_index)
      {
        double costOfStep = costmap_->getCost(new_index);
        TotalCost += costOfStep;
        if (costOfStep >= obstacleTH)
        {
          encoutered_obstacle = true;
          break;
        }
      }
      prev_index=new_index;
    }
    return TotalCost;
  }

  nav_msgs::msg::Path rrtstar::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only except start position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner will only except goal position from %s frame",
          global_frame_.c_str());
      return global_path;
    }
    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;
    double width = costmap_->getSizeInMetersX();
    double height = costmap_->getSizeInMetersY();
    int width_cell = costmap_->getSizeInCellsX();
    int height_cell = costmap_->getSizeInCellsY();
    double orgX = costmap_->getOriginX();
    double orgY = costmap_->getOriginY();
    unsigned char test_cost = 255;
    unsigned int minx = width_cell, miny = height_cell, maxx = 0, maxy = 0;
    mPerCellX = width / width_cell;
    mPerCellY = height / height_cell;

    for (int i = 0; i < width_cell; i++)
    {
      for (int j = 0; j < height_cell; j++)
      {
        unsigned char cost = costmap_->getCost(i, j);
        if (cost != 255)
        {
          if (i < minx)
            minx = i;
          if (j < miny)
            miny = j;
          if (i > maxx)
            maxx = i;
          if (j > maxy)
            maxy = j;
        }
      }
    }
    std::uniform_real_distribution<double> unifX(minx * mPerCellX, maxx * mPerCellX);
    std::uniform_real_distribution<double> unifY(miny * mPerCellY, maxy * mPerCellY);
    std::default_random_engine re;
    RCLCPP_INFO(node_->get_logger(), "minx: %u, minY%u, maxx:%u, maxy:%u", minx, miny, maxx, maxy);
    std::vector<Vertex> Verticies;
    Vertex Start(start.pose.position.x - orgX, start.pose.position.y - orgY, 0, -1);
    Verticies.push_back(Start);
    Vertex End(goal.pose.position.x - orgX, goal.pose.position.y - orgY, INT_MAX, -1);
    //Vertex midPoint((Start.x+End.x)/2,(Start.y+End.y)/2,255,0);
    //Verticies.push_back(midPoint);
    // int Endidx = costmap_->getIndex(End.x / mPerCellX, End.y / mPerCellY);
    // int EndCost=costmap_->getCost(Endidx);
    // RCLCPP_INFO(node_->get_logger(), "End Indes: %d End Cost: %d",Endidx,EndCost);
    bool encountered_obstacle=false;
    int cost=costBeetweanPoints(Start.x,Start.y,End.x,End.y,encountered_obstacle);
    RCLCPP_INFO(node_->get_logger(), "dist: %d obstacle: %d",cost,encountered_obstacle);
    //RCLCPP_INFO(node_->get_logger(), "Generating points");
    for (int i = 0; i < 10000; i++)
    {
      double rX = unifX(re);
      double rY = unifY(re);
     // RCLCPP_INFO(node_->get_logger(), "rX: %f, rY%f", rX, rY);
      int idx = costmap_->getIndex(rX / mPerCellX, rY / mPerCellY);
      if (costmap_->getCost(idx) >= obstacleTH)
      {
        continue;
      }
      int bestCost = INT_MAX;
      int Closest = -1;
      int iter = 0;
      for (auto &element : Verticies)
      {
        bool encountered_obstacle=false;
        int cost=costBeetweanPoints(rX,rY,element.x,element.y,encountered_obstacle);
        //RCLCPP_INFO(node_->get_logger(), "calculated cost %d",cost);
        if(encountered_obstacle)
        {
          iter++;
          continue;
        }
        //RCLCPP_INFO(node_->get_logger(), "past obstacle");
        // RCLCPP_INFO(node_->get_logger(),"eligible");
        double Totalcost = cost + element.Cost;
        //RCLCPP_INFO(node_->get_logger(), "calculated cost %f",Totalcost);
        if (Totalcost < bestCost)
        {
          bestCost = Totalcost;
          Closest = iter;
        }
        iter++;
      }
      //RCLCPP_INFO(node_->get_logger(), "best cost %d", bestCost);
      if (Closest != -1)
      {
        Vertex NewVertex(rX, rY, bestCost, Closest);
        Verticies.push_back(NewVertex);
        bool encountered_obstacle=false;
        int cost=costBeetweanPoints(rX,rY,End.x,End.y,encountered_obstacle);
        if (encountered_obstacle)
        {
          continue;
        }
        double Totalcost = cost + NewVertex.Cost;
        if (Totalcost < End.Cost)
        {
          End.Parent = Verticies.size();
          End.Cost = Totalcost;
        }
      }
    }
    // Vertex* currentPoint=&End;
    Vertex CurElement = End;
    RCLCPP_INFO(node_->get_logger(), "len (%ld)", Verticies.size());
    //std::vector<int> path_idx;
    int counter=0;
    while (CurElement.Parent != -1)
    {
      //RCLCPP_INFO(node_->get_logger(), "path idx (%d)", CurElement.Parent);
      Vertex nextElement=Verticies[CurElement.Parent];
      // RCLCPP_INFO(
      // node_->get_logger(), "idx (%d)",Curindx);
      int total_number_of_loop = std::hypot(
                                     CurElement.x - nextElement.x,
                                     CurElement.y - nextElement.y) /interpolation_resolution_;
      double x_increment = (CurElement.x-nextElement.x) / total_number_of_loop;
      double y_increment = (CurElement.y-nextElement.y) / total_number_of_loop;

      for (int i = 0; i < total_number_of_loop; ++i)
      {
        geometry_msgs::msg::PoseStamped pose;
        // RCLCPP_INFO(node_->get_logger(), "Poin (%f)",PrevElement.x + orgX + x_increment * i);
        pose.pose.position.x = CurElement.x  +orgX+ x_increment * i;
        pose.pose.position.y = CurElement.y  +orgY+ y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
        CurElement = nextElement;
      }
      RCLCPP_INFO(node_->get_logger(), "Counter (%d)", counter);
      counter++;
    }
    return global_path;
  }

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtstar_planner::rrtstar, nav2_core::GlobalPlanner)

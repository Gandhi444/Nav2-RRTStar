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
    unsigned char obstacleTH = 255;
    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;
    double widht = costmap_->getSizeInMetersX();
    double height = costmap_->getSizeInMetersY();
    int widht_cell = costmap_->getSizeInCellsX();
    int height_cell = costmap_->getSizeInCellsY();
    double orgX = costmap_->getOriginX();
    double orgY = costmap_->getOriginY();
    unsigned char test_cost=255;
    for(int i=0 ;i<height_cell;i++)
    {
      for(int j=0 ;j<widht_cell;j++)
      {
        unsigned char new_cost=costmap_->getCost(i,j);
        if(test_cost>new_cost)
        {
          test_cost=new_cost;
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(),"test cost %u",test_cost);


    //costmap_->saveMap("savedmap.pgm");
    // int cesl_size=costmap_->getSizeInCellsX();
    // RCLCPP_INFO(node_->get_logger(),"cell szie %d",cesl_size);
    // RCLCPP_INFO(node_->get_logger(),"cell szie %f",widht);
    // unsigned char* map=costmap_->getCharMap();
    // int map_size =costmap_->getIndex(widht,height);
    // RCLCPP_INFO(node_->get_logger(),"map size %d",map_size);
    // unsigned char min=255,max=0;
    // for(int i;i<147456+100;i++)
    // {
    //   unsigned char value=map[i];
    //   if(value<min)
    //   {
    //     min=value;
    //   }
    //   if(max<value)
    //   {
    //     max=value;
    //   }
    // }
    // RCLCPP_INFO(node_->get_logger(),"min %u max %u",min,max);

    // std::cout<<"ite:"<<orgX<<"random:"<<orgY<<std::endl;
    // std::cout<<"ite:"<<height+orgY<<"random:"<<widht+orgX<<std::endl;
    //  std::random_device rd;
    //  std::mt19937 gen(rd());
    // std::uniform_real_distribution<> distr(widht+orgX,height+orgY);
    std::vector<Vertex> Verticies;
    Vertex Start(start.pose.position.x - orgX, start.pose.position.y - orgY, 0, -1);
    // RCLCPP_INFO(node_->get_logger(),"Start addres %p",&Start);
    // Vertex End(goal.pose.position.x-orgX,goal.pose.position.y-orgY,DBL_MAX,-1);
    Verticies.push_back(Start);
    Vertex End(goal.pose.position.x - orgX, goal.pose.position.y - orgY, DBL_MAX, 0);
    // RCLCPP_INFO(node_->get_logger(),"End addres %p",&Verticies[0]);
    for (int i = 0; i < 1000; i++)
    {
      float r1 = static_cast<int>(rand()) / (static_cast<int>(RAND_MAX / (height_cell)));
      float r2 = static_cast<int>(rand()) / (static_cast<int>(RAND_MAX / (widht_cell)));
      std::pair<double, double> point(r1, r2);
      int idx = costmap_->getIndex(point.first, point.second);
      
      // std::cout<<"ite:"<<i<<"random:"<<r1<<std::endl;
      // std::cout<<(int)costmap_->getCost(idx)<<std::endl;
      //RCLCPP_INFO(node_->get_logger(),"cost %u",(int)costmap_->getCost(idx));
      if (costmap_->getCost(idx) >= obstacleTH)
      {
        continue;
      }
      double bestCost = DBL_MAX;
      int Closest = -1;
      int iter = 0;
      for (auto &element : Verticies)
      {

        int total_number_of_loop = std::hypot(
                                       element.x - point.first, element.y - point.second) /
                                   interpolation_resolution_;
        double x_increment = abs(element.x - point.first) / total_number_of_loop;
        double y_increment = abs(element.y - point.second) / total_number_of_loop;
        double costBeetweenPoints = 0;
        bool eligible = true;
        for (int j = 0; j < total_number_of_loop; ++j)
        {
          double position_x = element.x + x_increment * j;
          double position_y = element.y + y_increment * j;
          // std::cout<<position_x<<"  "<<position_y<<std::endl;
          double costOfStep = costmap_->getCost(costmap_->getIndex(position_x, position_y));
          // std::cout<<"after cost"<<std::endl;
          costBeetweenPoints += costOfStep;
          if (costOfStep >= obstacleTH)
          {
            eligible = false;
            break;
          }
        }
        if (!eligible)
        {
          
          continue;
        }
        //RCLCPP_INFO(node_->get_logger(),"eligible");
        double Totalcost = costBeetweenPoints + element.Cost;
        if (Totalcost < bestCost)
        {
          bestCost = Totalcost;
          Closest = iter;
        }
        iter++;
      }
      if (Closest != -1)
      {
        // RCLCPP_INFO(node_->get_logger(),"Current point %d",Closest);
        Vertex NewVertex(point.first, point.second, bestCost, Closest);
        // RCLCPP_INFO(node_->get_logger(),"Current point %p",&Verticies[Closest]);
        //RCLCPP_INFO(node_->get_logger(),"new");
        Verticies.push_back(NewVertex);
        int total_number_of_loop = std::hypot(
                                       End.x - point.first, End.y - point.second) /
                                   interpolation_resolution_;
        double x_increment = (End.x - point.first) / total_number_of_loop;
        double y_increment = (End.y - point.second) / total_number_of_loop;
        double costBeetweenPoints = 0;
        bool eligible = true;
        for (int j = 0; j < total_number_of_loop; ++j)
        {
          double position_x = End.x + x_increment * j;
          double position_y = End.y + y_increment * j;
          double costOfStep = costmap_->getCost(costmap_->getIndex(position_x, position_y));
          costBeetweenPoints += costOfStep;
          if (costOfStep >= obstacleTH)
          {
            eligible = false;
            break;
          }
        }
        if (!eligible)
        {
          continue;
        }
        double Totalcost = costBeetweenPoints + NewVertex.Cost;
        if (Totalcost < End.Cost)
        {
          End.Parent = Verticies.size();
          End.Cost = Totalcost;
        }
      }
    }
    // Vertex* currentPoint=&End;
    int Curindx = End.Parent;
    Vertex PrevElement = End;
          RCLCPP_INFO(
      node_->get_logger(), "len (%ld)",Verticies.size());
    while (Curindx != -1)
    {
      Vertex currentPoint = Verticies[Curindx];
      // RCLCPP_INFO(
      // node_->get_logger(), "idx (%d)",Curindx);
      int total_number_of_loop = std::hypot(
                                     currentPoint.x - PrevElement.x,
                                     currentPoint.y - PrevElement.y) /
                                 interpolation_resolution_;
      double x_increment = (currentPoint.x - PrevElement.x) / total_number_of_loop;
      double y_increment = (currentPoint.y - PrevElement.y) / total_number_of_loop;

      for (int i = 0; i < total_number_of_loop; ++i)
      {
        geometry_msgs::msg::PoseStamped pose;
        // RCLCPP_INFO(node_->get_logger(), "Poin (%f)",PrevElement.x + orgX + x_increment * i);
        pose.pose.position.x = currentPoint.x + orgX - x_increment * i;
        pose.pose.position.y = currentPoint.y + orgY - y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
        PrevElement = currentPoint;
        Curindx = currentPoint.Parent;
      }
    }
    return global_path;
  }

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtstar_planner::rrtstar, nav2_core::GlobalPlanner)

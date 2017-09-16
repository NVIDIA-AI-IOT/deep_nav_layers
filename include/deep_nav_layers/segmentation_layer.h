// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#ifndef SEGMENTATION_LAYER_H_
#define SEGMENTATION_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

namespace segmentation_layer
{

  class SegmentationLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  SegmentationLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();

private:
  std::set<int> obstacle_ids;
  std::set<int> path_ids;
  float x_range;
  float y_range;
  float m_per_pixel;
  int costmap_height;
  int costmap_width;
  bool new_data;
  cv::Mat warped;
  cv::Mat cropped;
  cv::Mat h;
  ros::Subscriber seg_sub_;
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  void segNetCb(const sensor_msgs::Image::ConstPtr &msg);
  void parseHomographyConstants(const std::string &homography_folder);
  void parseIntSet(const std::string &raw_list, std::set<int> &int_set);
};
}
#endif

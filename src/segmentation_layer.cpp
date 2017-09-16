// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <deep_nav_layers/segmentation_layer.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>

PLUGINLIB_EXPORT_CLASS(segmentation_layer::SegmentationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace segmentation_layer
{

SegmentationLayer::SegmentationLayer() {}

void SegmentationLayer::onInitialize()
{
  std::string segmentation_data;
  std::string homography_folder;
  std::string obstacle_ids_str;
  std::string path_ids_str;

  
  ros::NodeHandle nh("~/" + name_);
  nh.param<std::string>("segmentation_data",  segmentation_data, "/segnet/network/output");
  nh.param<std::string>("homography_folder",  homography_folder, "/home/nvidia/deep_nav_layers/calibrate_homography");
  nh.param<std::string>("obstacle_ids_str",  obstacle_ids_str, "14 15"); // zero-indexed IDs to be marked as an obstacle
  nh.param<std::string>("path_ids_str",  path_ids_str, "3"); // zero-indexed ID to clear obstacles
  nh.param<float>("x_range",  x_range, 1.0); // meters to the left/right of the bot to modify costmap
  nh.param<float>("y_range",  y_range, 1.0); // meters in front of the bot to modify costmap

  parseIntSet(obstacle_ids_str, obstacle_ids);
  parseIntSet(path_ids_str, path_ids);

  parseHomographyConstants(homography_folder);
  
  current_ = true;
  new_data = false;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SegmentationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  seg_sub_  = nh.subscribe<sensor_msgs::Image>(segmentation_data, 1, &SegmentationLayer::segNetCb, this);

  cv::FileStorage fs(homography_folder + "/homography.xml", cv::FileStorage::READ);
  fs["homography"] >> h;
  std::cout << "Loaded homography: " << h << std::endl;
  fs.release();
}

void SegmentationLayer::parseHomographyConstants(const std::string &homography_folder)
{
  cv::FileStorage fs = cv::FileStorage(homography_folder + "/parameters.yml", cv::FileStorage::READ);
  costmap_height = fs["costmap_height"];
  costmap_width = fs["costmap_width"];
  m_per_pixel = fs["m_per_pixel"];
}


void SegmentationLayer::parseIntSet(const std::string &raw_list, std::set<int> &int_set)
{
  std::stringstream ss(raw_list);

  int i;
  while (ss >> i)
  {
    int_set.insert(i);

    if (ss.peek() == ' ')
      ss.ignore();
  }
}

void SegmentationLayer::segNetCb(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::warpPerspective(cv_ptr->image, warped, h, cv::Size(costmap_width, costmap_height));
  
  int x_range_pixels = x_range/m_per_pixel;
  int y_range_pixels = y_range/m_per_pixel;
  // crop projection to only go a user defined number of meters in x and y direction
  cv::Rect ROI = cv::Rect(warped.cols/2 - x_range_pixels, warped.rows/2 - y_range_pixels, x_range_pixels*2, y_range_pixels*2);
  cropped = cv::Mat(warped, ROI); // note that this is just a reference
  new_data = true;
}
  
void SegmentationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
	    master->getOriginX(), master->getOriginY());
}
  
  // allows the plugin to dynamically change the configuration of the costmap
void SegmentationLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

  // determines the area of the costmap that is potentially going to be changed
void SegmentationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (!new_data)
    return;
  
  // convert to degrees and adjust starting point
  double angle = (robot_yaw*180)/M_PI - 90;

  cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(cropped.cols/2, cropped.rows/2), angle, 1);
  cv::Mat overlay;

  cv::warpAffine(cropped, overlay, rotation_matrix, cropped.size());
  cv::Point2f origin = cv::Point2f(overlay.cols/2, overlay.rows/2);
  
  for(int y=0; y<overlay.rows; y++){
      for(int x=0; x<overlay.cols; x++){
	int value = overlay.at<unsigned char>(cv::Point(x,y));
        if (obstacle_ids.find(value) != obstacle_ids.end()) {
	  
	  // shift over point so origin of image is at (0,0)
	  double mark_x = robot_x + (x - origin.x)*m_per_pixel;
	  double mark_y = robot_y + (origin.y - y)*m_per_pixel;

	  unsigned int mx, my;
	  if(worldToMap(mark_x, mark_y, mx, my)){
	    setCost(mx, my, LETHAL_OBSTACLE);
	  }
	}
        if (path_ids.find(value) != path_ids.end()) {
	  double mark_x = robot_x + (x - origin.x)*m_per_pixel;
	  double mark_y = robot_y + (origin.y - y)*m_per_pixel;
	  unsigned int mx, my;
	  if(worldToMap(mark_x, mark_y, mx, my)){
	    setCost(mx, my, FREE_SPACE);
	  }
	}
      }
  }

  // REVIEW: potentially make this configurable, or calculated?
  *min_x = -20; // 20 meters, max size
  *min_y = -20;
  *max_x = 20;
  *max_y = 20;

  new_data = false;
}

  // actually update the costs within the bounds
void SegmentationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
	continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace

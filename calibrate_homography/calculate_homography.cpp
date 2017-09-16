// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <opencv2/opencv.hpp>
#include <assert.h>

using namespace std;
using namespace cv;

class HomographyCalculator
{
private:
  int image_height, image_width;
  int seg_height, seg_width;
  int costmap_height, costmap_width;
  float m_per_pixel;
  string homography_file_path;
  
public:
  /*
    HomographyCalculator: load all the appropriate parameters
  */
  HomographyCalculator(const char *parameters_file_path)
  {
    FileStorage fs = FileStorage(parameters_file_path, FileStorage::READ);
    fs["image_height"] >> image_height;
    fs["image_width"] >> image_width;
    fs["seg_height"] >> seg_height;
    fs["seg_width"] >> seg_width;
    fs["costmap_height"] >> costmap_height;
    fs["costmap_width"] >> costmap_width;
    fs["m_per_pixel"] >> m_per_pixel;
    fs["homography_file_path"] >> homography_file_path;
  }

  /*
    inputImageToSegmentation: transform points from input image into segmentation output
    img_pts - coordinates in the input image
    seg_pts - points representing the same coordinates in the segmentation output
  */
  void inputImageToSegmentation(const vector<Point2f> &img_pts, vector<Point2f> &seg_pts)
  {
    for (vector<Point2f>::const_iterator it = img_pts.begin(); it != img_pts.end(); ++it) {
      Point2f pt = *it;
      float new_x = (pt.x * seg_width)/image_width; // shrink x
      float new_y = (pt.y * seg_height)/image_height; // shrink y
      seg_pts.push_back(Point2f(new_x, new_y));
    }
  }
  
  /* 
     coordToCostmap: transform points from real world coordinates to pixels on costmap
     coord_pts - coordinates of rectangle as distances from robot in meters
     costmap_pts - pixel locations of points, with robot at the center of the image
  */
  void coordToCostmap(const vector<Point2f> &coord_pts, vector<Point2f> &costmap_pts)
  {
    // place the origin at the center of the image
    // (note that the bottom half is blank, but may be filled with future rotations)
    Point2f origin = Point2f(costmap_width/2, costmap_height/2); 
    for (vector<Point2f>::const_iterator it = coord_pts.begin(); it != coord_pts.end(); ++it) {
      Point2f pt = *it;
      float new_x = origin.x + pt.x/m_per_pixel;
      float new_y = origin.y - pt.y/m_per_pixel; // since greater pixel value means down
    
      assert(new_x < costmap_width && new_y < costmap_height); // check point is valid
    
      costmap_pts.push_back(Point2f(new_x, new_y));
    }
  }

  /* readPoints: a helper function to convert {x: , y: } points into Point2f
     raw_pts - original FileNode list of points
     final_pts - a reference to the vector of Point2f to be filled
  */
  void readPoints(const FileNode &raw_pts, vector<Point2f> &final_pts)
  {
    for (FileNodeIterator it = raw_pts.begin(); it != raw_pts.end(); ++it)
    {
      Point2f new_point = Point2f((*it)["x"], (*it)["y"]);
      final_pts.push_back(new_point);
    }
  }

  /* writeHomography: calculates the homography between the set of points given and writes
     it to a file (as given by the class variable homography_file_path)
     seg_pts - the points in the segmentation output
     costmap_pts - the corresponding points in the costmap
  */
  void writeHomography(const vector<Point2f> &seg_pts, const vector<Point2f> &costmap_pts)
  {
    Mat h = findHomography(seg_pts, costmap_pts);
    cout << "Calculated homography: " << h << endl;
  
    FileStorage fs(homography_file_path, FileStorage::WRITE);
    fs << "homography" << h;
    fs.release();
    cout << "Wrote to file: " << homography_file_path << endl;
  }

};

/* 
   main: calculate the homography matrix and write it a file
*/
int main(int argc, char** argv)
{
  // load the calculator with the appropriate values
  HomographyCalculator h_calc = HomographyCalculator("../parameters.yml");

  // read in the points (note that the points could come from anywhere)
  FileStorage fs = FileStorage("../calibration_points.yml", FileStorage::READ);

  // transform the points based on parameters
  vector<Point2f> img_pts, seg_pts;
  h_calc.readPoints(fs["img_pts"], img_pts);
  h_calc.inputImageToSegmentation(img_pts, seg_pts);

  // transform the points based on parameters
  vector<Point2f> coord_pts, costmap_pts;
  h_calc.readPoints(fs["coord_pts"], coord_pts);
  h_calc.coordToCostmap(coord_pts, costmap_pts);
  // calculate the final homography
  h_calc.writeHomography(seg_pts, costmap_pts);
  
  return 0;
}

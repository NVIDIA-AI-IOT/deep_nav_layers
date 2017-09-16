// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class HomographyTester
{
private:
  int costmap_height, costmap_width;
  Mat homography;
  Mat seg_data;
  
public:
  /* 
     HomographyTester: load all the parameters and images
  */
  HomographyTester(const char *parameters_file_path)
  {
    FileStorage fs = FileStorage(parameters_file_path, FileStorage::READ);
    fs["costmap_height"] >> costmap_height;
    fs["costmap_width"] >> costmap_width;
    string homography_file_path;
    fs["homography_file_path"] >> homography_file_path;
    loadHomography(homography_file_path);
    
    string seg_data_path;
    fs["seg_data_path"] >> seg_data_path;
    seg_data = imread(seg_data_path, CV_LOAD_IMAGE_COLOR);
    cout << "Loaded segmentation data from " << seg_data_path << endl;
  }

  /*
    loadHomography: read the homography into memory
    homography_file_path - the path to read from
  */
  void loadHomography(const string &homography_file_path)
  {
    FileStorage fs(homography_file_path, FileStorage::READ);
    fs["homography"] >> homography;
    cout << "Loaded homography from " << homography_file_path << endl;
    fs.release();
  }

  /*
    showResults(): show the effect of the homography on the sample data
  */
  void showResults()
  {
    // show original
    imshow("Original Segmentation Data", seg_data);
    
    Mat warped;
    // apply homography
    warpPerspective(seg_data, warped, homography, Size(costmap_height, costmap_width));

    // resize warped image for easier viewing
    Mat scaled_warped;
    resize(warped, scaled_warped, Size(), 5, 5);

    imshow("Warped Segmentation Data", warped);
    imshow("Warped Segmentation Data (scaled)", scaled_warped);
  }
    
};
  
int main(int argc, char **argv)
{
  // load the calculator with the appropriate values
  HomographyTester h_test = HomographyTester("../parameters.yml");

  h_test.showResults();
  waitKey(0);

  return 0;
}

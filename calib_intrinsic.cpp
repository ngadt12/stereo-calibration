#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "cxxopts/cxxopts.hpp"

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > image_points;
vector< Point2f > corners;
vector< vector< Point2f > > left_img_points;

Mat img, gray;
Size im_size;

void setup_calibration(int board_width, int board_height, int num_imgs, 
                       float square_size, const char* imgs_directory, const char* imgs_filename,
                       const char* extension) {
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int k = 1; k <= num_imgs; k++) {
    char img_file[100];
    sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, k, extension);
    img = imread(img_file, CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    bool found = false;
    found = cv::findChessboardCorners(img, board_size, corners,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if (found)
    {
      cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                   TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray, board_size, corners, found);
    }
    
    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      cout << k << ". Found corners!" << endl;
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char *argv[])
{
  int board_width, board_height, num_imgs;
  float square_size;
  string imgs_directory;
  string imgs_filename;
  string extension;  
  string out_file;

  try {
	  cxxopts::Options options(argv[0], "calibrate single camera's instrinsic parameters");

	  options.add_options()
		  ("w,board_width", "Checkerboard width", cxxopts::value<int>(board_width), "NUM")
		  ("h,board_height", "Checkerboard height", cxxopts::value<int>(board_height), "NUM")
		  ("n,num_imgs", "Number of images to process", cxxopts::value<int>(num_imgs), "NUM")
		  ("s,square_size", "Checkerboard square size (i.e. in mm)", cxxopts::value<float>(square_size), "NUM")
		  ("d,imgs_directory", "Directory containing images", cxxopts::value<string>(imgs_directory), "STR")
		  ("i,imgs_filename", "Image filename", cxxopts::value<string>(imgs_filename), "STR")
		  ("e,extension", "Image extension", cxxopts::value<string>(extension), "STR")
		  ("o,out_file", "Output calibration filename (YML)", cxxopts::value<string>(out_file), "STR")
		  ("help", "print help")
		  ;

	  options.parse(argc, argv);
	  if (options.count("help"))
	  {
	    cout << options.help() << endl;
        exit(0);
	  }
	  cxxopts::check_required(options, {"w","h","n","s","d","i","e","o"}); // throws exception if any not present
  }
  catch (const cxxopts::OptionException& e)
  {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }

  setup_calibration(board_width, board_height, num_imgs, square_size,
                   imgs_directory.c_str(), imgs_filename.c_str(), extension.c_str());

  printf("Starting Calibration\n");
  Mat K;
  Mat D;
  vector< Mat > rvecs, tvecs;
  int flag = 0;
  flag |= CV_CALIB_FIX_K4;
  flag |= CV_CALIB_FIX_K5;
  calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);

  cout << "Calibration error: " << computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D) << endl;

  FileStorage fs(out_file, FileStorage::WRITE);
  fs << "K" << K;
  fs << "D" << D;
  fs << "board_width" << board_width;
  fs << "board_height" << board_height;
  fs << "square_size" << square_size;
  printf("Done Calibration\n");

  return 0;
}

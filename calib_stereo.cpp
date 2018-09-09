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
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2;

void load_image_points(int board_width, int board_height, int num_imgs, float square_size,
                      const char* leftimg_dir, const char* rightimg_dir, const char* leftimg_filename, const char* rightimg_filename) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;

  for (int i = 1; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    sprintf(left_img, "%s%s%d.jpg", leftimg_dir, leftimg_filename, i);
    sprintf(right_img, "%s%s%d.jpg", rightimg_dir, rightimg_filename, i);
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found2 = cv::findChessboardCorners(img2, board_size, corners2,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found1)
    {
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
    }
    if (found2)
    {
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
    }

    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found1 && found2) {
      cout << i << ". Found corners!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}

int main(int argc, char *argv[])
{
  string leftcalib_file;
  string rightcalib_file;
  string leftimg_dir;
  string rightimg_dir;
  string leftimg_filename;
  string rightimg_filename;
  string out_file;
  int num_imgs;

  try {
	  cxxopts::Options options(argv[0], "calibrate stereo cameras");
	  options.add_options()
			( "n,num_imgs","Number of checkerboard images to process", cxxopts::value<int>(num_imgs),"NUM" )
			( "u,leftcalib_file","Left camera calibration",cxxopts::value<string>(leftcalib_file),"STR" )
			( "v,rightcalib_file","Right camera calibration",cxxopts::value<string>(rightcalib_file),"STR" )
			( "L,leftimg_dir","Directory containing left images",cxxopts::value<string>(leftimg_dir),"STR" )
			( "R,rightimg_dir","Directory containing right images",cxxopts::value<string>(rightimg_dir),"STR" )
			( "l,leftimg_filename","Left image prefix",cxxopts::value<string>(leftimg_filename),"STR" )
			( "r,rightimg_filename","Right image prefix",cxxopts::value<string>(rightimg_filename),"STR" )
			( "o,out_file","Output calibration filename (YML)",cxxopts::value<string>(out_file),"STR" )
		    ( "h,help", "print help")
			;

	  options.parse(argc, argv);
	  if (options.count("help"))
	  {
	    cout << options.help() << endl;
        exit(0);
	  }
	  cxxopts::check_required(options, {"n","u","v","L","R","l","r","o"}); // throws exception if any not present
  }
  catch (const cxxopts::OptionException& e)
  {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }

  FileStorage fsl(leftcalib_file, FileStorage::READ);
  FileStorage fsr(rightcalib_file, FileStorage::READ);

  load_image_points(fsl["board_width"], fsl["board_height"], num_imgs, fsl["square_size"],
                   leftimg_dir.c_str(), rightimg_dir.c_str(), leftimg_filename.c_str(), rightimg_filename.c_str());

  printf("Starting Calibration\n");
  Mat K1, K2, R, F, E;
  Vec3d T;
  Mat D1, D2;
  fsl["K"] >> K1;
  fsr["K"] >> K2;
  fsl["D"] >> D1;
  fsr["D"] >> D2;
  int flag = 0;
  flag |= CV_CALIB_FIX_INTRINSIC;
  
  cout << "Read intrinsics" << endl;
  
  stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F);

  cv::FileStorage fs1(out_file, cv::FileStorage::WRITE);
  fs1 << "K1" << K1;
  fs1 << "K2" << K2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  
  printf("Done Calibration\n");

  printf("Starting Rectification\n");

  cv::Mat R1, R2, P1, P2, Q;
  //flag = CV_CALIB_ZERO_DISPARITY;
  flag = 0; double alpha = -1;
  stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, flag, alpha);

  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  return 0;
}

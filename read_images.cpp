#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "cxxopts/cxxopts.hpp"

using namespace std;
using namespace cv;

int x = 0;

int main(int argc, char *argv[])
{
  string imgs_directory = "";
  string extension = "";
  int im_width, im_height =0;

	try {
		cxxopts::Options options(argv[0], "read images from cameras, save to disk");

		options.add_options()
			("w,img_width", "image width, e.g. 640", cxxopts::value<int>(im_width), "NUM")
			("h,img_height", "image height, e.g. 480", cxxopts::value<int>(im_height), "NUM")
			("d,imgs_directory", "directory to save images, e.g. \"C:/imgs/\"", cxxopts::value<string>(imgs_directory), "STR")
			("e,extension", "image extension, e.g. \"jpg\"", cxxopts::value<string>(extension), "STR")
			("help", "print help")
			;

		options.parse(argc, argv);
		if (options.count("help"))
		{
		  cout << options.help() << endl;
		  exit(0);
		}
		cxxopts::check_required(options,{"w","h","d","e"});
  }
  catch (const cxxopts::OptionException& e)
  {
    cout << "error parsing options: " << e.what() << endl;
    exit(1);
  }

  cout << "img w x h: " << im_width << " x " << im_height << "\noutput dir: " << imgs_directory << ", img ext: " << extension << endl;

  VideoCapture cap1(0);
  VideoCapture cap2(1);
  if (cap1.isOpened() && cap2.isOpened()) {
	  cout << "opened 2 cameras" << endl;
  }
  else {
	  cout << "error opening cameras!" << endl;
	  return 1;
  }

  Mat img1, img_res1, img2, img_res2;
  while (1) {
	cap1 >> img1;
	cap2 >> img2;
    resize(img1, img_res1, Size(im_width, im_height));
    resize(img2, img_res2, Size(im_width, im_height));
    imshow("IMG1", img_res1);
    imshow("IMG2", img_res2);
    if (waitKey(30) > 0) {
      x++;
      char filename1[200], filename2[200];
      sprintf(filename1, "%sleft%d.%s", imgs_directory.c_str(), x, extension.c_str());
      sprintf(filename2, "%sright%d.%s", imgs_directory.c_str(), x, extension.c_str());
      cout << "Saving img pair " << x << endl;
      imwrite(filename1, img_res1);
      imwrite(filename2, img_res2);
    }
  }
  return 0;
}

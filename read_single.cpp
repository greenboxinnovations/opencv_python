#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>
#include <iostream>

// mkdir
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


using namespace std;
using namespace cv;

int x = 0;

int main(int argc, char const *argv[])
{
  string imgs_directory;
  string extension;
  int im_width, im_height;

  im_width = 1920;
  im_height = 1080;

  if(argc < 2){
    cout << "usage:\n./read_single <img_dir+/> <left/right>\nspace to click, enter to clear chessboard" << endl;
    return 0;
  }


  imgs_directory  = argv[1];
  string type = argv[2];
  extension = "jpg";

  bool show_chessboard = false;


  struct stat st = {0};

  if (stat(imgs_directory.c_str(), &st) == -1) {
      mkdir(imgs_directory.c_str(), 0700);
  }


  const string WINDOW_NAME1 = "123";  
  namedWindow(WINDOW_NAME1, WINDOW_NORMAL);  
  resizeWindow(WINDOW_NAME1, 640, 480);  



  const string CHESSWINDOW = "chessboard";

  bool board_found = false;



  // get ip from file
  FileStorage fs("ip.yml", FileStorage::READ);
  string ip;
  fs["ip"] >> ip;
  const string CAM_IP = ip;

  VideoCapture cap(CAM_IP);

  Mat img1, img_res1, img2, img_res2;
  Mat img_chess;


  while (1) {
    cap >> img1;    

    resize(img1, img_res1, Size(im_width, im_height));    
    imshow(WINDOW_NAME1, img_res1);    



    switch(waitKey(30)) {

      // escape
      case 27:
        if(!show_chessboard){
          return 0;  
        }        
        break;


      // space
      case 32:
        if(!show_chessboard){
          show_chessboard = true;
          cout << "space"<< endl;

          
          cv::cvtColor(img_res1, img_chess, COLOR_BGR2GRAY);          




          namedWindow(CHESSWINDOW, 0);
          resizeWindow(CHESSWINDOW, 1280, 960);  

          vector<Point2f> corners;
          Size boardSize = Size(13,9);
          Mat img_chess_bgr;
          
          board_found = findChessboardCorners(img_chess, boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

          
          cvtColor(img_chess, img_chess_bgr, COLOR_GRAY2BGR);
          drawChessboardCorners(img_chess_bgr, boardSize, corners, board_found);



          putText(img_chess_bgr, "Y=confirm. N=discard", Point2f(30,30), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255), 2);
          imshow(CHESSWINDOW, img_chess_bgr);
        }        
        break;


      // y 
      case 121:

        if(show_chessboard){

          show_chessboard = false;
          board_found = false;
          destroyWindow(CHESSWINDOW);


          x++;
          char filename1[200], filename2[200];
          if(type == "left"){
            sprintf(filename1, "%sleft%d.%s", imgs_directory.c_str(), x, extension.c_str());  
          }else{
            sprintf(filename1, "%sright%d.%s", imgs_directory.c_str(), x, extension.c_str());
          }
          cout << "Saving img pair " << x << endl;
          imwrite(filename1, img_res1);
        }
        break;

      // n 
      case 110:
        if(show_chessboard){

            show_chessboard = false;
            board_found = false;
            destroyWindow(CHESSWINDOW);
        }
        break;

    }
  }
  return 0;
}

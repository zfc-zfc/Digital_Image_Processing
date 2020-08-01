#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#define LINEAR_X 0.0
using namespace std;
using namespace cv;

Mat HSV_Img;
//Color Segmentation Code
int main(int argc, char **argv)
{
	VideoCapture capture;
        capture.open(0);//打开zed相机

	ROS_WARN("*****START");
	ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);	//定义速度发布器

	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}
	waitKey(1000);
	Mat frIn;//当前帧图片

	while (1)
	{
		capture.read(frIn);
		if(frIn.empty())
		{
			break;
		}

    	cvtColor(frIn,HSV_Img,COLOR_BGR2HSV); //将图片转换为HSI
    	imshow("HSV_Image",HSV_Img);
		//分割蓝色
		Mat devide_blue_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		for(int i=0;i<HSV_Img.rows;i++){
			for(int j=0;j<HSV_Img.cols;j++){
				if(HSV_Img.at<Vec3b>(i,j)[0] > 100 && HSV_Img.at<Vec3b>(i,j)[0] < 124){		
					if(HSV_Img.at<Vec3b>(i,j)[1] > 43 && HSV_Img.at<Vec3b>(i,j)[1] < 255){
						if(HSV_Img.at<Vec3b>(i,j)[2] > 46 && HSV_Img.at<Vec3b>(i,j)[2] < 255){
							devide_blue_Img.at<unsigned char>(i,j) = 255;	
						}
					}
				}
				else
					devide_blue_Img.at<unsigned char>(i,j) = 0;		
			}
		}
		Mat ndevide_blue_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		GaussianBlur(devide_blue_Img, ndevide_blue_Img, Size(5,5),0);
		vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;
		findContours(ndevide_blue_Img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
		Scalar color = Scalar(255,255,255);
		drawContours( ndevide_blue_Img, contours, -1, color, 2, 5, vector<Vec4i>(), 2, Point() );

		//分割绿色
		Mat devide_green_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		for(int i=0;i<HSV_Img.rows;i++){
			for(int j=0;j<HSV_Img.cols;j++){
				if(HSV_Img.at<Vec3b>(i,j)[0] > 35 && HSV_Img.at<Vec3b>(i,j)[0] < 77){		
					if(HSV_Img.at<Vec3b>(i,j)[1] > 43 && HSV_Img.at<Vec3b>(i,j)[1] < 255){
						if(HSV_Img.at<Vec3b>(i,j)[2] > 46 && HSV_Img.at<Vec3b>(i,j)[2] < 255){
							devide_green_Img.at<unsigned char>(i,j) = 255;	
						}
					}
				}
				else
					devide_green_Img.at<unsigned char>(i,j) = 0;		
			}
		}
		Mat ndevide_green_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		GaussianBlur(devide_green_Img, ndevide_green_Img, Size(5,5),0);
		findContours(ndevide_green_Img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
		drawContours( ndevide_green_Img, contours, -1, color, 2, 5, vector<Vec4i>(), 2, Point() );

		//分割红色
		Mat devide_red_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		for(int i=0;i<HSV_Img.rows;i++){
			for(int j=0;j<HSV_Img.cols;j++){
				if((HSV_Img.at<Vec3b>(i,j)[0] > 156 && HSV_Img.at<Vec3b>(i,j)[0] < 180)||(HSV_Img.at<Vec3b>(i,j)[0] > 0 && HSV_Img.at<Vec3b>(i,j)[0] < 10)){		
					if(HSV_Img.at<Vec3b>(i,j)[1] > 43 && HSV_Img.at<Vec3b>(i,j)[1] < 255){
						if(HSV_Img.at<Vec3b>(i,j)[2] > 46 && HSV_Img.at<Vec3b>(i,j)[2] < 255){
							devide_red_Img.at<unsigned char>(i,j) = 255;	
						}
					}
				}
				else
					devide_red_Img.at<unsigned char>(i,j) = 0;		
			}
		}
		Mat ndevide_red_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		GaussianBlur(devide_red_Img, ndevide_red_Img, Size(5,5),0);
		findContours(ndevide_red_Img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
		drawContours( ndevide_red_Img, contours, -1, color, 2, 5, vector<Vec4i>(), 2, Point() );

		//分割橙色
		Mat devide_orange_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		for(int i=0;i<HSV_Img.rows;i++){
			for(int j=0;j<HSV_Img.cols;j++){
				if(HSV_Img.at<Vec3b>(i,j)[0] > 11 && HSV_Img.at<Vec3b>(i,j)[0] < 25){		
					if(HSV_Img.at<Vec3b>(i,j)[1] > 43 && HSV_Img.at<Vec3b>(i,j)[1] < 255){
						if(HSV_Img.at<Vec3b>(i,j)[2] > 46 && HSV_Img.at<Vec3b>(i,j)[2] < 255){
							devide_orange_Img.at<unsigned char>(i,j) = 255;	
						}
					}
				}
				else
					devide_orange_Img.at<unsigned char>(i,j) = 0;		
			}
		}
		Mat ndevide_orange_Img(HSV_Img.rows, HSV_Img.cols, CV_8UC1);	
		GaussianBlur(devide_orange_Img, ndevide_orange_Img, Size(5,5),0);
		findContours(ndevide_orange_Img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
		drawContours( ndevide_orange_Img, contours, -1, color, 2, 5, vector<Vec4i>(), 2, Point() );
    	imshow("frIn",frIn);
		imshow("devide_blue_Img",ndevide_blue_Img);
		imshow("devide_green_Img",ndevide_green_Img);
		imshow("devide_red_Img",ndevide_red_Img);
		imshow("devide_orange_Img",ndevide_orange_Img);
		waitKey(5);
	}
	return 0;
}



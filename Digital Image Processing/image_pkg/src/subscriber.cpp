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
#define LINEAR_X 0
using namespace cv;
//Histogram equalization code
int main(int argc, char **argv)
{
	VideoCapture capture;
        capture.open(0);//打开zed相机

	
	ROS_WARN("*****START");
	ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
        ros::NodeHandle n;

        // ros::Rate loop_rate(10);//定义速度发布频率
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器


	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}
	waitKey(1000);
	Mat frame;//当前帧图片
	int nFrames = 0;//图片帧数
	int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
	int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高

	const int L = 256;//灰度个数
	int grayNum[L] = { 0 };//每个灰度下像素点的个数
	double grayFreq[L] = { 0 };	//每个灰度出现的频率
	double graySumFre[L] = { 0 };	//累计密度
	int nGrayVal[L] = { 0 };	//均衡化后的灰度值
	int gray_size = 0;  //像素总数
	//int grayMax = 0;	//最大灰度值
	memset(grayNum, 0,sizeof(grayNum));//清零

	while (ros::ok())
	{
		capture.read(frame);
		if(frame.empty())
		{
			break;
		}

		Mat frIn = frame(cv::Rect(0, 0, frame.cols, frame.rows));//截取zed的左目图片
		Mat frGray;
		

		// 此处增加直方图均衡化
		//求P(r) //2*frIn/(frame.cols*frame.rows)?
		//求Sk=T(r) //求和，换算
		//叠加求均衡化概率

		cvtColor(frIn, frGray, CV_RGB2GRAY);//RGB彩色图转换成Gray灰度图
		Mat frNew(frGray.rows, frGray.cols, CV_8UC1);
		gray_size = frGray.rows*frGray.cols;
		for(int i=0;i<frIn.rows;i++)//遍历图像，对应灰度值下grayNum++
		{
			for(int j=0;j<frIn.cols;j++)
			{
				grayNum[frGray.at<uchar>(i,j)]++;
			}
		}

		Mat dstHist;       
		int dims = 1;
		float hranges[] = {0, 256};
		const float *ranges[] = {hranges};   // 这里需要为const类型
		int size = 256;
		int channels = 0;
	
		//计算图像的直方图
		calcHist(&frGray, 1, &channels, Mat(), dstHist, dims, &size, ranges); 
		Mat dstImage(size, size, CV_8U, Scalar(0));
		//获取最大值和最小值
		double minValue = 0;
		double maxValue = 0;
		minMaxLoc(dstHist,&minValue, &maxValue, 0, 0);  //  在cv中用的是cvGetMinMaxHistValue
		
		//绘制出直方图
		//saturate_cast函数的作用即是：当运算完之后，结果为负，则转为0，结果超出255，则为255。
		int hpt = saturate_cast<int>(0.9 * size);
		for(int i = 0; i < 256; i++)
		{
			float binValue = dstHist.at<float>(i);           //   注意hist中是float类型   
			//拉伸到0-max
			int realValue = saturate_cast<int>(binValue * hpt/maxValue);
			line(dstImage,Point(i, size - 1),Point(i, size - realValue),Scalar(255));
		}

		for(int i=0;i<L;i++)	//计算对应频率
		{
			grayFreq[i] = (double)(grayNum[i]) / gray_size;
		}
		graySumFre[0] = grayFreq[0];
		for(int i=1;i<L;i++)	//计算累计频率
		{
				graySumFre[i] = graySumFre[i-1]+grayFreq[i];

			nGrayVal[i] = (int)(L*graySumFre[i] + 0.5);
		}
		
		for(int i=0;i<frIn.rows;i++)	//给新图片写如入像素点
		{
			for(int j=0;j<frIn.cols;j++)
			{
				frNew.at<uchar>(i,j) = nGrayVal[frGray.at<uchar>(i,j)];
			}
		}

		Mat dstHist1;       
		int dims1 = 1;
		float hranges1[] = {0, 256};
		const float *ranges1[] = {hranges1};   // 这里需要为const类型
		size = 256;
		channels = 0;
	
		//计算图像的直方图
		calcHist(&frNew, 1, &channels, Mat(), dstHist1, dims1, &size, ranges1); 
		Mat dstImage1(size, size, CV_8U, Scalar(0));
		//获取最大值和最小值
		minValue = 0;
		maxValue = 0;
		minMaxLoc(dstHist1,&minValue, &maxValue, 0, 0);  //  在cv中用的是cvGetMinMaxHistValue
		
		//绘制出直方图
		//saturate_cast函数的作用即是：当运算完之后，结果为负，则转为0，结果超出255，则为255。
		int hpt1 = saturate_cast<int>(0.9 * size);
		for(int i = 0; i < 256; i++)
		{
			float binValue1 = dstHist1.at<float>(i);           //   注意hist中是float类型   
			//拉伸到0-max
			int realValue1 = saturate_cast<int>(binValue1 * hpt1/maxValue);
			line(dstImage1,Point(i, size - 1),Point(i, size - realValue1),Scalar(255));
		}

        imshow("1",frIn);
		imshow("2",frGray);
		imshow("3",frNew);
		imshow("4",dstImage);
		imshow("5",dstImage1);


		memset(grayFreq, 0,sizeof(grayFreq));//清零
		memset(grayNum, 0,sizeof(grayNum));//清零
		memset(graySumFre, 0,sizeof(graySumFre));//清零
		memset(nGrayVal, 0,sizeof(nGrayVal));//清零
		
 		printf("frGray.rows=%d\n",frGray.rows);
		printf("frGray.cols=%d\n",frGray.cols);
		for(int k=0;k<320;k=k+30)
		{
			printf("frGray.at<uchar>(k,k)=%d\n",frGray.at<uchar>(k,k));
		}			
		
		printf("frNew.at<uchar>(20,20)=%d\n",frGray.at<uchar>(20,20));
		printf("grayFreq[50]=%f\n",grayFreq[50]);
		printf("\n");


		geometry_msgs::Twist cmd_red;

		// 车的速度值设置
		cmd_red.linear.x = LINEAR_X;
		cmd_red.linear.y = 0;
		cmd_red.linear.z = 0;
		cmd_red.angular.x = 0;
		cmd_red.angular.y = 0;
		cmd_red.angular.z = 0.2;
	
		pub.publish(cmd_red);

		ros::spinOnce();
//		loop_rate.sleep();
		waitKey(5);

	}


	return 0;
}




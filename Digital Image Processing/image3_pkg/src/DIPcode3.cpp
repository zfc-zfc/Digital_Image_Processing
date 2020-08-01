#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#define LINEAR_X 0
using namespace cv;
using namespace std;
const double PI = acos(-1.0);//反三角函数
//Canny Eage Extraction Code
//对图像进行高斯模糊处理
void Gaussian(Mat input, Mat output, double sigma)
{
    //设计高斯滤波模板。当sigma固定时，模板尺寸越大，图像越模糊
    int MaskSize=3;
    int center_k=MaskSize/2;
    int center_l=MaskSize/2;
    double sum=0;   //模板权值之和
    double mask[MaskSize][MaskSize];
    //生成高斯滤波模板，对mask数组进行赋值操作
    for (int k = 0; k < MaskSize; k++ ){
        for (int l = 0; l < MaskSize; l++){
            mask[k][l] = exp( -(1.0)*( ((k-center_k)*(k-center_k)+(l-center_l)*(l-center_l))/(2.0*sigma*sigma)) );
            sum += mask[k][l];
        }
    }
    //归一化模板权值
    for (int i = 0; i < MaskSize; i++){
        for (int j = 0; j < MaskSize; j++){
            mask[i][j] /= sum;
        }
    }
    //将模板函数与图像进行卷积
    for(int i=0;i<input.rows;i++){
        for(int j=0;j<input.cols;j++){
            double sum = 0;
            for (int k = 0; k <MaskSize; k++){
                for (int l = 0; l < MaskSize; l++){
                    if(i+(k-center_k)>=0 && j+(l-center_l)>=0)    //判断。相当于将图像外面一圈像素的灰度值置零
                    sum = sum+input.at<uchar>(i+(k-center_k),j+(l-center_l))*mask[k][l];
                }
            }
            //对输出图像重新赋值
            output.at<uchar>(i,j)=sum;
        }
    }
}

//计算梯度的幅值和角度，采用sobel算子
void Gradient(Mat input, Mat output_Mag,Mat output_Ang)
{
    int sobel_x[3][3]={{-1,-2,-1},{0,0,0},{1,2,1}};//水平方向的sobel算子
    int sobel_y[3][3]={{-1,0,1},{-2,0,2},{-1,0,1}};//竖直方向的sobel算子
    for(int i=0;i<input.rows;i++){
        for(int j=0;j<input.cols;j++){
            float sum_x = 0;
            float sum_y = 0;
            for (int k = 0; k < 3; k++){
                for (int l = 0; l < 3; l++){
                    if(i+(k-1)>=0 && j+(l-1)>=0)
                    sum_x = sum_x+input.at<uchar>(i+(k-1),j+(l-1))*sobel_x[k][l];
                    sum_y = sum_y+input.at<uchar>(i+(k-1),j+(l-1))*sobel_y[k][l];
                }
            }
            //对输出图像重新赋值,计算梯度算子的幅值
            output_Mag.at<uchar>(i,j)=sqrt(pow((sum_x), 2) + pow((sum_y), 2));
            output_Ang.at<uchar>(i,j)=atan(sum_y/sum_x);
        }
    }
}


void Suppress(Mat input_Mag, Mat input_Ang, Mat output) {
    
    int size=3;
    for (int i = 0; i < input_Mag.rows; i++)
        for (int j = 0; j < input_Mag.cols; j++) {
            float maxP = 0.0f;
            float phi = input_Ang.at<uchar>(i,j);
            float mag = input_Mag.at<uchar>(i,j);
            if(abs(phi) < PI / 8) { // Horizontal contour
                for(int s = i - size / 2; s - size < i - size / 2; s++) {
                    if (s < 0 || s >= input_Mag.rows) continue;
                    maxP = maxP < input_Mag.at<uchar>(s,j) ? input_Mag.at<uchar>(s, j) : maxP;
                }
            } else if (phi > 3 * PI / 8 || phi < -3 * PI / 8) { // Vertical contour
                for(int t = j - size / 2; t - size < j - size / 2; t++) {
                    if (t < 0 || t >= input_Mag.cols) continue;
                    maxP = maxP < input_Mag.at<uchar>(i,t) ? input_Mag.at<uchar>(i, t) : maxP;
                }
            } else if(phi > 0) { //+45 degree
                for(int k = -size / 2; k - (-size / 2) < size; k++) {
                    if(i + k < 0 || i + k >= input_Mag.rows || j + k < 0 || j + k >= input_Mag.cols) continue;
                    maxP = maxP < input_Mag.at<uchar>(i + k, j + k) ? input_Mag.at<uchar>(i + k, j + k) : maxP;
                }
            } else {	// -45 degree
                for(int k = -size / 2; k - (-size / 2) < size; k++) {
                    if(i + k < 0 || i + k >= input_Mag.rows || j - k < 0 || j - k >= input_Mag.cols) continue;
                    maxP = maxP < input_Mag.at<uchar>(i + k, j - k) ? input_Mag.at<uchar>(i + k, j - k) : maxP;
                }
            }
            float outij = output.at<uchar>(i, j);
            outij = abs(outij - maxP) < 0.000001 ? outij  : 0;
            output.at<uchar>(i, j)=outij;
        }
    return ;
}


//滞后阈值,先用30和90试一下
void Delay(Mat input, Mat output)
{
    int Threshold_H=200;
    int Threshold_L=70;
    for(int i=1;i<input.rows-1;i++){
        for(int j=1;j<input.cols-1;j++){
            if(input.at<uchar>(i,j) < Threshold_L)
                output.at<uchar>(i,j)=0;
            else if((input.at<uchar>(i,j) >= Threshold_L) && (input.at<uchar>(i,j) <= Threshold_H)){
                int judge=0;
                for(int k=-1;k<2;k++){
                    for(int l=-1;l<2;l++){
                        if ( input.at<uchar>(i+k,j+l) > Threshold_H){
                            judge=1;break;
                        }
                    }
                }
                if(judge=0)
                    output.at<uchar>(i,j)=0;
                else
                    output.at<uchar>(i,j)=255;
            }
            else
                output.at<uchar>(i,j)=255;
        }
    }
}


int main(int argc, char **argv)
{

    VideoCapture capture;
        capture.open(0);//打开 zed 相机

	ROS_WARN("*****START");
	ros::init(argc,argv,"trafficLaneTrack");
        ros::NodeHandle n;

    ros::Rate loop_rate(10);
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开\n");
        return 0;
    }
	waitKey(1000);
    Mat frame;
	while (ros::ok())
	{
        /*
        capture.read(frame);
        if(frame.empty())
        {
            break;
        }
        Mat frIn1 = frame(cv::Rect(0, 0, frame.cols, frame.rows));//截取 zed 的左目图片
        Mat frIn;
        cvtColor(frIn1, frIn, CV_RGB2GRAY);//RGB彩色图转换成Gray灰度图
        */
		
        Mat frIn1 = imread("/home/fangcheng/Library/wsx2.jpg",1);
            if(frIn1.empty())
		{
			break;
		}
        imshow("frIn1",frIn1);
        Mat frIn;
        cvtColor(frIn1, frIn, CV_RGB2GRAY);//RGB彩色图转换成Gray灰度图
        
        




        /*
        Mat frameHSV;
        Mat mask(frame.rows, frame.cols, CV_8UC1);  // 2值掩膜
        Mat dstTemp1(frame.rows, frame.cols, CV_8UC1);  
        Mat dstTemp2(frame.rows, frame.cols, CV_8UC1);  
        cvtColor(frIn1, frameHSV, CV_BGR2HSV ); 
        // 对HSV空间进行量化，得到2值图像，亮的部分为手的形状  
        inRange(frameHSV, Scalar(0,30,30), Scalar(40,170,256), dstTemp1);  
        inRange(frameHSV, Scalar(156,30,30), Scalar(180,170,256), dstTemp2);  
        bitwise_or(dstTemp1, dstTemp2, mask);  
        imshow("mask",mask);

        vector< vector<Point> > contours;
	    vector<Vec4i> hierarchy;
	    findContours(mask,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
        Mat result(mask.size(),CV_8U,Scalar(0));
        drawContours(result,contours,-1,Scalar(255),2);
        imshow("result",result);

        */



        //高斯模糊处理
        Mat frGaussian = frIn.clone();
        Gaussian(frIn,frGaussian,5);
        //计算梯度的幅值和角度
        Mat frMagnitude = frGaussian.clone();
        Mat frAngle = frGaussian.clone();
        Gradient(frGaussian,frMagnitude,frAngle);
        //非极大值抑止
        Mat frSuppressed=frMagnitude.clone();
        Suppress(frMagnitude,frAngle,frSuppressed);
        //滞后阈值
        Mat frDelay=frSuppressed.clone();
        Delay(frSuppressed,frDelay);

        //opencv Canny
        Mat Canny=frIn.clone();
        cv::Canny(frIn,Canny,80,160);
        
        imshow("Canny",Canny);
        imshow("frIn",frIn);
		//imshow("frGray",frGray);
        imshow("frGaussian",frGaussian);
        imshow("frMagnitude",frMagnitude);
        imshow("frSuppressed",frSuppressed);
        imshow("frDelay",frDelay);
        
		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}




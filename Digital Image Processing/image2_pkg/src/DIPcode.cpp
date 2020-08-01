#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"
#define LINEAR_X 0
using namespace cv;
using namespace std;
Mat dstImage;   //频谱图
const double PI = acos(-1.0);//反三角函数

//////////////////////滤波//////////////////
// 空域高斯滤波器函数，在模板大小固定，sigma越大越模糊
void Gaussian(Mat input, Mat output, double sigma)
{
    //设计高斯滤波模板。当sigma固定时，模板尺寸越大，图像越模糊
    int MaskSize=6;   //模板的大小为３x３
    //找到模板的中心位置。对于３x3模板，中心为（１，１）
    int center_k=MaskSize/2;
    int center_l=MaskSize/2;
    double sum=0;   //模板权值之和
    double mask[MaskSize][MaskSize];   //用一个二维数组来表示高斯滤波模板
    //生成高斯滤波模板，对mask数组进行赋值操作
    for (int k = 0; k < MaskSize; k++ ){
        for (int l = 0; l < MaskSize; l++){
            //后面进行归一化，这部分可以不用乘以系数0.5 *pi*(sigma*sigma)
            mask[k][l] = exp( -(1.0)*( ((k-center_k)*(k-center_k)+(l-center_l)*(l-center_l))/(2.0*sigma*sigma)) );
            sum += mask[k][l];
        }
    }
    //归一化模板权值
    for (int i = 0; i < MaskSize; i++){
        for (int j = 0; j < MaskSize; j++){
            mask[i][j] /= sum;
            //printf(" [%.15f] ", mask[i][j]);
        }
        //printf("\n");
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

//频域滤波,参考了csdn
Mat freqfilt(Mat &scr, Mat &blur)
{
        //创建一个Mat类的数组，每一个元素都是一个Mat
        Mat plane[] = { scr, Mat::zeros(scr.size() , CV_32FC1) }; //创建通道，存储dft后的实部与虚部（CV_32F，必须为单通道数）
        Mat complexIm;
        merge(plane, 2, complexIm);//合并通道 （把两个矩阵合并为一个2通道的Mat类容器）
        dft(complexIm, complexIm);//进行傅立叶变换，结果保存在自身
        //***************中心化********************
        split(complexIm, plane);//分离通道（数组分离）

        int cx = plane[0].cols / 2; int cy = plane[0].rows / 2;          //以下的操作是移动图像(零频移到中心)
        //元素坐标表示为(cx,cy)
        Mat part1_r(plane[0], Rect(0, 0, cx, cy));
        Mat part2_r(plane[0], Rect(cx, 0, cx, cy));
        Mat part3_r(plane[0], Rect(0, cy, cx, cy));
        Mat part4_r(plane[0], Rect(cx, cy, cx, cy));
        //左上与右下交换位置(实部)
        Mat temp;
        part1_r.copyTo(temp);
        part4_r.copyTo(part1_r);
        temp.copyTo(part4_r);
        //右上与左下交换位置(实部)
        part2_r.copyTo(temp);
        part3_r.copyTo(part2_r);
        temp.copyTo(part3_r);
        //元素坐标(cx,cy)
        Mat part1_i(plane[1], Rect(0, 0, cx, cy));
        Mat part2_i(plane[1], Rect(cx, 0, cx, cy));
        Mat part3_i(plane[1], Rect(0, cy, cx, cy));
        Mat part4_i(plane[1], Rect(cx, cy, cx, cy));
        //左上与右下交换位置(虚部)
        part1_i.copyTo(temp);
        part4_i.copyTo(part1_i);
        temp.copyTo(part4_i);

        //右上与左下交换位置(虚部)
        part2_i.copyTo(temp);
        part3_i.copyTo(part2_i);
        temp.copyTo(part3_i);

        //*****************滤波器函数与DFT结果的乘积****************
        Mat blur_r, blur_i, BLUR;
        multiply(plane[0], blur, blur_r); //滤波（实部与滤波器模板对应元素相乘）
        multiply(plane[1], blur, blur_i); //滤波（虚部与滤波器模板对应元素相乘）
        Mat plane1[] = { blur_r, blur_i };
        merge(plane1, 2, BLUR);//实部与虚部合并

        //*********************得到原图频谱图***********************************
        magnitude(plane[0], plane[1], plane[0]);	//获取幅度图像，0通道为实部通道，1为虚部
        plane[0] += Scalar::all(1);			//傅立叶变换后的图片不好分析，进行对数处理
                                                        //plane[0]每一个元素加１，防止出现log0
        log(plane[0], plane[0]);			// float型的灰度空间为[0，1]
        normalize(plane[0], plane[0], 1, 0, CV_MINMAX); //线性归一化，便于显示，最常用

        idft(BLUR, BLUR);  //idft结果也为复数
        split(BLUR, plane);//分离通道，主要获取通道
        magnitude(plane[0], plane[1], plane[0]);    //求幅值(模)
        normalize(plane[0], plane[0], 1, 0, CV_MINMAX);  //归一化便于显示
        return plane[0];//返回参数
}

//*****************理想低通滤波器***********************
Mat ideal_lbrf_kernel(Mat &src, float sigma)
{
        Mat ideal_low_pass(src.size(), CV_32FC1);
        float d0 = sigma;  //半径D0越小，模糊越大；半径D0越大，模糊越小
        for (int i = 0; i<src.rows; i++) {
                for (int j = 0; j<src.cols; j++) {
                        double d = sqrt(pow((i - src.rows / 2), 2) + pow((j - src.cols / 2), 2)); //计算到中心点的距离
                        if (d <= d0) {
                            ideal_low_pass.at<float>(i, j) = 1;
                        }
                        else {
                            ideal_low_pass.at<float>(i, j) = 0;
                        }
                }
        }
        return ideal_low_pass;
}

Mat ideal_Low_Pass_Filter(Mat &src, float sigma)   //sigma越小越模糊
{
        int M = getOptimalDFTSize(src.rows);
        int N = getOptimalDFTSize(src.cols);
        Mat padded;            //调整图像加速傅里叶变换
        copyMakeBorder(src, padded, 0, M - src.rows, 0, N - src.cols, BORDER_CONSTANT, Scalar::all(0));//扩充边界
        padded.convertTo(padded, CV_32FC1); //将图像转换为float型
        Mat ideal_kernel = ideal_lbrf_kernel(padded, sigma);//理想低通滤波器
        Mat result = freqfilt(padded, ideal_kernel);
        return result;
}
//建立复数结构体
struct CComplex {
        CComplex(float r, float ir):real(r),irreal(ir){}
        CComplex():real(0),irreal(0){}
        float real;
        float irreal;
};

//乘法运算
CComplex eMultiply(const CComplex& a, const CComplex& b) {
        return CComplex(a.real*b.real - a.irreal*b.irreal, a.real*b.irreal + a.irreal*b.real);
}
//加法运算
CComplex eAdd(const CComplex& a, const CComplex& b) {
        return CComplex(a.real + b.real, a.irreal + b.irreal);
}

//求二进制逆序数,得到傅里叶变换蝶形运算需要的原离散序列的顺序
int reverse_bit(int num, int len)
{
        int i, bit;
        unsigned new_num = 0;
        for (i = 0; i < len; i++){
                bit = num & 1;
                new_num <<= 1;
                new_num = new_num | bit;
                num >>= 1;
        }
        return new_num;
}

//判断是否是2的整数次方
int if_binaryNum(int length) {
        int num = 0;
        while (length != 1) {
                if (length % 2 == 0) {
                        length = length / 2;
                        num++;
                }
                else {
                        return -1;
                }
        }
        return num;
}

//将非2的整数次方边长的图片裁剪为2的整数次方
Mat binarylizeImage(Mat image) {
        float c = image.cols, r = image.rows;
        int cn = 0, rn = 0, cnew = 2, rnew = 2;
        while (c / 2 > 1) { c = c / 2; cn++;}
        while (r / 2 > 1) { r = r / 2; rn++;}
        while (cn > 0) { cnew = cnew * 2; cn--;}
        while (rn > 0) { rnew = rnew * 2; rn--;}
        resize(image, image, Size(cnew, rnew));
        return image;
}

//FFT
void fastFuriorTransform(Mat image) {
        int lengthC = image.cols;
        int lengthR = image.rows;
        int numC, numR;
        vector <CComplex> resultE;
        Mat furiorResultF = Mat(image.cols, image.rows, CV_32FC1);
        //映射表
        vector <int> mappingC;
        vector <int> mappingR;
        //W值表
        vector <CComplex> mappingWC;
        vector <CComplex> mappingWR;

        //判断输入图片边长是否是2的n次方，如果不符合，调整image大小
        numC = if_binaryNum(lengthC);
        numR = if_binaryNum(lengthR);
        if (numC == -1 || numR == -1) {
                fastFuriorTransform(binarylizeImage(image));
                return;
        }

        //构造映射表
        for (int c = 0; c < image.cols; c++) {
                mappingC.push_back(0);
        }
        for (int r = 0; r < image.rows; r++) {
                mappingR.push_back(0);
        }
        for (int c = 0; c < image.cols; c++) {
                mappingC.at(reverse_bit(c, numC)) = c;  //reverse_bit(c, numC)给出原序列元素应该放置在蝶形运算中的位置
        }
        for (int r = 0; r < image.rows; r++) {
                mappingR.at(reverse_bit(r, numR)) = r;
        }

        //构造W表,相当于旋转因子的x次方
        for (int i = 0; i < lengthC / 2; i++) {
                CComplex w(cosf(2 * PI / lengthC * i), -1 * sinf(2 * PI / lengthC * i));
                mappingWC.push_back(w);
        }
        for (int i = 0; i < lengthR / 2; i++) {
                CComplex w(cosf(2 * PI / lengthR * i), -1 * sinf(2 * PI / lengthR * i));
                mappingWR.push_back(w);
        }

        //初始化
        for (int r = 0; r < lengthR; r++) {
                for (int c = 0; c < lengthC; c++) {
                        //利用映射表，并且以0到1区间的32位浮点类型存储灰度值
                        //实部为灰度值，虚部为0
                        CComplex w((float)image.at<uchar>(mappingR.at(r), mappingC.at(c)) / 255, 0);  //使用mappingR或mappingC得到需要的蝶形坐标
                        resultE.push_back(w);
                }
        }

        //循环计算每行
        for (int r = 0; r < lengthR; r++) {
                //循环更新resultE中当前行的数值，即按照蝶形向前层层推进
                for (int i = 0; i < numC; i++) {   //总共有numC级蝶形结构，numC=log2N
                        int combineSize = 2 << i;   //代表第m级蝶形运算的个数为2^m
                        vector<CComplex> newRow;
                        //按照2,4,8,16...为单位进行合并，并更新节点的值
                        for (int j = 0; j < lengthC; j = j + combineSize) {  // j是当前的蝶形单元，j+combineSize代表跳到下一个蝶形运算
                                int n;
                                for (int k = 0; k < combineSize; k++) {  //在一个蝶形中运算
                                        if (k < (combineSize / 2)) {  //在前N/2， X1()+WX2()
                                                int w = k * lengthC / combineSize;
                                                n = k + j + r*lengthC;   // k + j是选出第r行(r*lengthC)的第j个蝶形运算单元
                                                newRow.push_back(eAdd(resultE.at(n), eMultiply(resultE.at(n + (combineSize >> 1)), mappingWC.at(w))));
                                        }
                                        else {   //在后N/2   X1()-WX2()
                                                int w = (k - (combineSize >> 1)) * lengthC / combineSize;
                                                n = k + j - (combineSize >> 1) + r*lengthC;
                                                newRow.push_back(eAdd(resultE.at(n), eMultiply(CComplex(-1, 0), eMultiply(resultE.at(n + (combineSize >> 1)), mappingWC.at(w)))));
                                        }

                                }
                        }
                        //用newRow来更新resultE中的值
                        for (int j = 0; j < lengthC; j++) {
                                int n = j + r*lengthC;
                                resultE.at(n) = newRow.at(j);
                        }
                        newRow.clear();
                }
        }

        //循环计算每列
        for (int c = 0; c < lengthC; c++) {
                for (int i = 0; i < numR; i++) {
                        int combineSize = 2 << i;
                        vector <CComplex> newColum;
                        for (int j = 0; j < lengthR; j = j + combineSize) {
                                int n;
                                for (int k = 0; k < combineSize; k++) {
                                        if (k < (combineSize >> 1)) {
                                                int w = k * lengthR / combineSize;
                                                n = (j + k) * lengthC + c;
                                                newColum.push_back(eAdd(resultE.at(n), eMultiply(resultE.at(n + (combineSize >> 1)*lengthC), mappingWR.at(w))));
                                        }
                                        else {
                                                int w = (k - (combineSize >> 1)) * lengthR / combineSize;
                                                n = (j + k - (combineSize >> 1)) * lengthC + c;
                                                newColum.push_back(eAdd(resultE.at(n), eMultiply(CComplex(-1, 0), eMultiply(resultE.at(n + (combineSize >> 1)*lengthC), mappingWR.at(w)))));
                                        }
                                }
                        }
                        //用newColum来更新resultE中的值
                        for (int j = 0; j < lengthR; j++) {
                                int n = j*lengthC + c;
                                resultE.at(n) = newColum.at(j);
                        }
                        newColum.clear();
                }
        }

        //结果存入一个vector<float>中
        float val_max, val_min;
        vector <float> amplitude;
        for (int r = 0; r < lengthR; r++) {
                for (int c = 0; c < lengthC; c++) {
                        CComplex e = resultE.at(r*lengthC + c);
                        float val = sqrt(e.real*e.real + e.irreal*e.irreal) + 1;
                        //对数尺度缩放
                        val = log(val);
                        amplitude.push_back(val);
                        if (c == 0 && r == 0) {
                                val_max = val;
                                val_min = val;
                        }
                        else {
                                if (val_max < val) val_max = val;
                                if (val_min > val) val_min = val;
                        }
                }
        }

        //将vector中的数据转存到Mat中，并归一化到0到255区间
        Mat fftResult = Mat(lengthC, lengthR, CV_8UC1);
        for (int i = 0; i < lengthR; i++) {
                for (int j = 0; j < lengthC; j++) {
                        int val = (int)((amplitude.at(i*lengthC + j) - val_min) * 255 / (val_max - val_min));
                        fftResult.at<uchar>(i, j) = val;
                }
        }
        //调整象限
        int cx = fftResult.cols / 2;
        int cy = fftResult.rows / 2;
        Mat q0(fftResult, Rect(0, 0, cx, cy));
        Mat q1(fftResult, Rect(cx, 0, cx, cy));
        Mat q2(fftResult, Rect(0, cy, cx, cy));
        Mat q3(fftResult, Rect(cx, cy, cx, cy));

        Mat tmp;
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);
        q1.copyTo(tmp);
        q2.copyTo(q1);
        tmp.copyTo(q2);

        dstImage = fftResult.clone();
}

//////////////////////形态学//////////////////
// 膨胀函数,仅处理二值图像？
void Dilate(Mat Src, Mat Dst)
{
    //定义膨胀结构元
    int Di_Size=3;
    for(int i=Di_Size/2;i<Src.rows-Di_Size/2;i++){
        for(int j=Di_Size/2;j<Src.cols-Di_Size/2;j++){
            int sum=0;
            for (int k = 0; k <Di_Size; k++){
                for (int l = 0; l < Di_Size; l++){
                    //膨胀要求图像区域的九个像素灰度值和模板有交集，有一个为0即可
                    //令图像区域九个像素块灰度值的和再除以255为sum，当sum<9，就表示不全为1，即有一个为0，击中
                    sum=sum+Src.at<uchar>(i-(k-Di_Size/2),j-(l-Di_Size/2))/255;
                }
            }
            if(sum<9)
                Dst.at<uchar>(i,j)=0;
            else
                Dst.at<uchar>(i,j)=255;
        }
    }
}

// 腐蚀函数
void Erode(Mat Src, Mat Dst)
{
    //定义腐蚀结构元
    int Er_Size=3;
    for(int i=Er_Size/2;i<Src.rows-Er_Size/2;i++){
        for(int j=Er_Size/2;j<Src.cols-Er_Size/2;j++){
            int sum=0;
            for (int k = 0; k <Er_Size; k++){
                for (int l = 0; l < Er_Size; l++){
                    //腐蚀要求图像区域的九个像素灰度值和模板完全匹配，即全为０
                    //令图像区域九个像素块灰度值的和为sum，当且仅当sum=0，才有完全匹配
                    sum=sum+Src.at<uchar>(i-(k-Er_Size/2),j-(l-Er_Size/2));
                }
            }
            if (sum==0)
                Dst.at<uchar>(i,j)=0;
            else
                Dst.at<uchar>(i,j)=255;
        }
    }
}

int main(int argc, char **argv)
{
    VideoCapture capture;
        capture.open(0);//打开 zed 相机ROS_WARN("*****START");

    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化 ROS 节点
        ros::NodeHandle n;

        ros::Rate loop_rate(10);//定义速度发布频率
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器

    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);
    Mat frame;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高

    while (ros::ok())
    {
        capture.read(frame);
        if(frame.empty())
        {
            break;
        }
        // Mat frIn = frame();
        //使用笔记本摄像头
        Mat frIn = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));//截取 zed 的左目图片
        Mat frGray;
        cvtColor(frIn, frGray, CV_RGB2GRAY);//RGB彩色图转换成Gray灰度图
        Mat frGaussian = frGray.clone();
        // 空域滤波函数
        Gaussian(frGray,frGaussian,200.0);
        //快速傅里叶变换
        Mat frFFT = frGray.clone();
        fastFuriorTransform(frFFT);
        ideal_lbrf_kernel(frFFT,200.0);
        Mat ideal_LP = ideal_Low_Pass_Filter(frGray,10);
        //对图像进行二值化,灰度值高于阈值的元素灰度值设为255(白色)，低于阈值的为0（黑色）
        Mat frThreshold=frGray.clone();
        cv::threshold(frGray, frThreshold, 125, 255, THRESH_BINARY);//第三个参数为阈值
        printf("threshold=%d\n",frThreshold.at<uchar>(200,200));//用于检测二值图像的灰度值
        // 膨胀函数
        Mat frDilation=frThreshold.clone();
        Dilate(frThreshold,frDilation);
        // 腐蚀函数
        Mat frErosion=frThreshold.clone();
        Erode(frThreshold,frErosion);
        imshow("frIn",frIn);
        imshow("frGray",frGray);
        imshow("frGaussian",frGaussian);
        imshow("dstImage",dstImage);
        imshow("ideal_LP",ideal_LP);
        imshow("frThreshold",frThreshold);
        imshow("frDilation",frDilation);
        imshow("frErosion",frErosion);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
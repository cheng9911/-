#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include <thread>
#include <chrono>
#include <zbar.h>
#include <unistd.h>
#include <opencv2/aruco.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <algorithm>
#include <QDebug>
#include "marker.h"

using namespace std;
using namespace Eigen;
using namespace ur_rtde;

//添加zbar名称空间
using namespace cv;

double distance_pxpy(Point P1, Point P2)
{
    float distance_p;
    double x1 = P1.x, y1 = P1.y;
    double x2 = P2.x, y2 = P2.y;
    distance_p = sqrt(pow(P2.x - P1.x, 2) + pow(P2.y - P1.y, 2));
    return distance_p;
}
QList<Point> Qlist_vector2Qlist(QList<Point> OutquadList, QList<vector<Point>> OutquadList_1, int num)
{
    Point mid;
    for (int i = 0; i < 4; i++)
    {
        mid.x = OutquadList_1[num][i].x;
        mid.y = OutquadList_1[num][i].y;
        OutquadList.append(mid);
        // cout<<OutquadList.size()<<endl;
    }

    return OutquadList;
}
//B为A的顺时针的下一个框
QList<Point> sortPoint_B(QList<Point> OutquadList, QList<Point> sortList, Point image_cen, Point A_cen)
{

    if (OutquadList.size() < 3)
    {
        std::cout << OutquadList.size() << std::endl;
        std::cout << "识别得到的角点组数缺失" << std::endl;
    }
    else
    {
        int point_3 = 0;
        int point_1 = 0;
        int point_0 = 0;
        int point_2 = 0;
        vector<double> add_point(4, 0);
        vector<double> add_point_1(2, 0);
        QList<Point> OutquadList_copy;
        double compare_min = distance_pxpy(image_cen, OutquadList[0]);
        double compare_max = distance_pxpy(image_cen, OutquadList[0]);
        for (int i = 0; i < 4; i++)
        {
            OutquadList_copy.push_back(OutquadList[i]);
            add_point[i] = distance_pxpy(image_cen, OutquadList[i]);
            if (add_point[i] < compare_min)
            {

                compare_min = add_point[i];
                point_3 = i;
            }
            if (add_point[i] > compare_max)
            {
                compare_max = add_point[i];
                point_1 = i;
            }
        }
        OutquadList_copy.removeAll(OutquadList[point_1]);

        OutquadList_copy.removeAll(OutquadList[point_3]);
        double compare_min_1 = distance_pxpy(A_cen, OutquadList_copy[0]);
        double compare_max_1 = distance_pxpy(A_cen, OutquadList_copy[0]);
        for (int j = 0; j < 2; j++)
        {
            add_point_1[j] = distance_pxpy(A_cen, OutquadList_copy[j]);
            if (add_point_1[j] < compare_min_1)
            {
                compare_min_1 = add_point_1[j];
                point_0 = j;
            }
            if (add_point_1[j] > compare_max_1)
            {
                compare_max_1 = add_point_1[j];
                point_2 = j;
            }
        }
        sortList.append(OutquadList_copy[point_0]);

        sortList.append(OutquadList[point_1]);
        sortList.append(OutquadList_copy[point_2]);
        sortList.append(OutquadList[point_3]);

        // cout<<"point_0"<<OutquadList_copy[point_0]<<endl;
        // cout<<"point_1"<<OutquadList[point_1]<<endl;
        // cout<<"point_2"<<OutquadList_copy[point_2]<<endl;
        // cout<<"point_3"<<OutquadList[point_3]<<endl;
    }
    return sortList;
}

//C为A的逆时针的下一个框
QList<Point> sortPoint_C(QList<Point> OutquadList, QList<Point> sortList, Point image_cen, Point A_cen)
{
    if (OutquadList.size() < 3)
    {
        std::cout << "识别得到的角点组数缺失" << std::endl;
    }
    else
    {
        int point_3 = 0;
        int point_1 = 0;
        int point_0 = 0;
        int point_2 = 0;
        vector<double> add_point(4, 0);
        vector<double> add_point_1(2, 0);
        QList<Point> OutquadList_copy;
        double compare_min = distance_pxpy(image_cen, OutquadList[0]);
        double compare_max = distance_pxpy(image_cen, OutquadList[0]);
        for (int i = 0; i < 4; i++)
        {
            OutquadList_copy.push_back(OutquadList[i]);
            add_point[i] = distance_pxpy(image_cen, OutquadList[i]);
            if (add_point[i] < compare_min)
            {

                compare_min = add_point[i];
                point_1 = i;
            }
            if (add_point[i] > compare_max)
            {
                compare_max = add_point[i];
                point_3 = i;
            }
        }
        OutquadList_copy.removeAll(OutquadList[point_1]);

        OutquadList_copy.removeAll(OutquadList[point_3]);
        double compare_min_1 = distance_pxpy(A_cen, OutquadList_copy[0]);
        double compare_max_1 = distance_pxpy(A_cen, OutquadList_copy[0]);
        for (int j = 0; j < 2; j++)
        {
            add_point_1[j] = distance_pxpy(A_cen, OutquadList_copy[j]);
            if (add_point_1[j] < compare_min_1)
            {
                compare_min_1 = add_point_1[j];
                point_0 = j;
            }
            if (add_point_1[j] > compare_max_1)
            {
                compare_max_1 = add_point_1[j];
                point_2 = j;
            }
        }
        sortList.append(OutquadList_copy[point_0]);
        sortList.append(OutquadList[point_1]);
        sortList.append(OutquadList_copy[point_2]);
        sortList.append(OutquadList[point_3]);
    }
    return sortList;
}

Point CrossPoint(Point P1, Point P2, Point P3, Point P4)
{
    Point pt;
    double x1 = P1.x, y1 = P1.y;
    double x2 = P2.x, y2 = P2.y;
    double x3 = P3.x, y3 = P3.y;
    double x4 = P4.x, y4 = P4.y;
    double D = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (D == 0)
    {
        cout << "000000000" << endl;
        pt.x = x1;
        pt.y = y3;
    }
    else
    {
        pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / D;
        pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / D;
    }
    return pt;
}

Mat rot_v2T(Mat R, Mat P)
{
    Mat T = (Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), P.at<double>(0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), P.at<double>(1),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), P.at<double>(2),
             0, 0, 0, 1);

    return T;
}
float getAngelOfTwoVector(Point2f pt1, Point2f pt2, Point2f c)
{
    //没有考虑存在的危险，atan2不存在
    float theta = atan2(pt1.x - c.x, pt1.y - c.y) - atan2(pt2.x - c.x, pt2.y - c.y);
    if (theta > CV_PI)
        theta -= 2 * CV_PI;
    if (theta < -CV_PI)
        theta += 2 * CV_PI;
    theta = theta * 180.0 / CV_PI;
    return theta;
}


int main(int argc, char *argv[])
{
    try
    {
        //摄像头调用
       
        cv::VideoCapture inputVideo(6); //如果只有1个摄像机，那么就是0，如果系统中有多个摄像机，那么只要将其向上增加即可。6为realsense
        inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

        //

        //机械臂连接,控制
        RTDEControlInterface rtde_control("192.168.3.101");
        RTDEReceiveInterface rtde_receive("192.168.3.101");
        int j = 1;
        vector<double> tvecs_sum(3, 0);
        vector<double> quat_sum(4, 0);

        //角点初始化
        float x_A = 0;
        float y_A = 0;
        float x_B = 0;
        float y_B = 0;
        float x_C = 0;
        float y_C = 0;
        float x_D = 0;
        float y_D = 0;
        int sum_corner = 0;
        int jump_front = 0;
        //

       double fx = 1364.61;
    double cx = 972;
    double fy = 1361.76;
    double cy = 556.603;

    double k1 = 0;
    double k2 = 0.;
    double p1 = 0;
    double p2 = 0;
    double k3 = 0;
        Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0.0, cx,
                            0.0, fy, cy,
                            0.0, 0.0, 1.0);
        Mat distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
        //相机标定的参数-------
        //二维码的边长
        double marker_size = 1030; //单位  cm
        while (inputVideo.grab())
        // while (true)
        {
            //image
            cv::Mat image;
            cv::Mat gray;
            cv::Mat threshold_output;
            // image = cv::imread("/home/sun/learn/learn_cmake/src/qrcode/src/1.png");
            inputVideo >> image;

            cv::cvtColor(image, gray, CV_BGR2GRAY);
            threshold(gray, threshold_output, 125, 255, THRESH_BINARY);
            Mat threshold_output_copy = threshold_output.clone();
            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
            morphologyEx(threshold_output_copy, threshold_output_copy, MORPH_OPEN, element);  //开运算
            morphologyEx(threshold_output_copy, threshold_output_copy, MORPH_CLOSE, element); //闭运算
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            Mat canny_output;
            Canny(threshold_output_copy, canny_output, 1, 3, 7, true); //Canny检测
            Mat image_canny = canny_output.clone();
            findContours(image_canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
            Mat Contours = Mat::zeros(image_canny.size(), CV_8UC1);
            //绘制 //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
            for (int i = 0; i < contours.size(); i++)
                for (int j = 0; j < contours[i].size(); j++)
                {
                    Point P = Point(contours[i][j].x, contours[i][j].y);
                    Contours.at<uchar>(P) = 255;
                }

            QList<vector<vector<Point>>> qrPointList; //每个节点表示一个回形点集
            // std::vector<std::vector<cv::Point>> qrPointList;
            vector<vector<Point>> qrPoint;
            Mat mat6 = Contours.clone();
            cvtColor(mat6, mat6, CV_GRAY2BGR);
            int Parindex[3] = {-1, -1, -1};
            for (int i = 0; i < contours.size(); i++)
            {
                if (hierarchy[i][3] != -1 && hierarchy[i][2] == -1)
                {
                    Parindex[0] = hierarchy[i][3];
                    if (hierarchy[(Parindex[0])][3] != -1)
                    {
                        Parindex[1] = hierarchy[(Parindex[0])][3];
                        if (hierarchy[(Parindex[1])][3] != -1)
                        {
                            Parindex[2] = hierarchy[(Parindex[1])][3];
                            if (hierarchy[(Parindex[2])][3] != -1)
                            {
                                if (!(i - 1 == Parindex[0] && Parindex[0] - 1 == Parindex[1] && Parindex[1] - 1 == Parindex[2]))
                                    continue; //都是独生轮廓
                                qrPoint.push_back(contours[i]);
                                qrPoint.push_back(contours[i - 2]);
                                qrPoint.push_back(contours[i - 4]);
                                for (int i = 0; i < qrPoint.size(); i++)
                                    for (int j = 0; j < qrPoint[i].size(); j++)
                                        circle(mat6, qrPoint[i][j], 2, Scalar(0, 255, 0), -1);
                                qrPointList.push_back(qrPoint);
                                qrPoint.clear();
                            }
                        }
                    }
                }
            }

            if (qrPointList.size() >= 3)
            {

                QList<Point> pointList;                             //存储角点中心
                QList<RotatedRect> RectList;                        //存储角元素最外层矩形
                QList<vector<Point>> OutquadList;                   //存储最外层拟合四边形角点
                vector<bool> qrPointListEnable(qrPointList.size()); //筛选时使用
                for (int L = 0; L < qrPointList.size(); L++)        //遍历每个可能的图元
                {
                    qrPoint = qrPointList.at(L);
                    vector<vector<Point>> contours_poly(qrPoint.size());
                    vector<RotatedRect> minRect(qrPoint.size()); //存储了嵌套的最小外接矩形*****
                    vector<Point2f> rect_center(qrPoint.size());
                    for (int i = 0; i < qrPoint.size(); i++)
                    {
                        approxPolyDP(Mat(qrPoint[i]), contours_poly[i], 5, true); //用指定精度逼近多边形曲线
                        minRect[i] = minAreaRect(Mat(qrPoint[i]));                //得到最小外接矩形
                        rect_center[i] = minRect[i].center;                       //得到最小外接矩形中心
                    }
                    //根据同心度筛选
                    for (int i = 0; i < minRect.size() - 1; i++)
                    {
                        Point P1 = Point(rect_center[i].x, rect_center[i].y);
                        Point P2 = Point(rect_center[i + 1].x, rect_center[i + 1].y);
                        float ConcenError_Set = (minRect[i].size.width + minRect[i].size.height) / 12; //***最大允差设定***
                        if (sqrt(pow(P1.x - P2.x, 2) + pow(P1.y - P2.y, 2)) > ConcenError_Set)
                        {
                            qrPointListEnable[L] = false;
                            break;
                        }
                        else
                            qrPointListEnable[L] = true;
                    }
                    if (!qrPointListEnable[L])
                        continue;

                    //根据三层周长比例进行筛选（3:5:7）
                    for (int i = 0; i < minRect.size() - 1; i++)
                    {
                        float circum1 = (minRect[i].size.width + minRect[i].size.height) * 2;
                        float circum2 = (minRect[i + 1].size.width + minRect[i + 1].size.height) * 2;
                        if (circum1 / circum2 >= 0.5 && circum1 / circum2 <= 0.8) //***周长比例设定***
                            qrPointListEnable[L] = true;
                        else
                        {
                            qrPointListEnable[L] = false;
                            break;
                        }
                    }
                    if (!qrPointListEnable[L])
                        continue;

                    //周长不能过小
                    for (int i = 0; i < minRect.size(); i++)
                    {
                        float circum = (minRect[i].size.width + minRect[i].size.height) * 2;
                        float circum_Set = 20; //***有效周长最小值设定***
                        if (circum >= circum_Set)
                            qrPointListEnable[L] = true;
                        else
                        {
                            qrPointListEnable[L] = false;
                            break;
                        }
                    }
                    if (!qrPointListEnable[L])
                        continue;

                    //筛选完毕！！！筛选出的个数可能为任意自然数

                    for (int i = 0; i < qrPoint.size(); i++)
                    {
                        Point2f rect_points[4];
                        minRect[i].points(rect_points);
                        if (i == 2)
                            RectList.push_back(minRect[i]); //RectList赋值   筛选后的最外层外接矩形
                        bool exsit = false;
                        Point P = Point(rect_center[i].x, rect_center[i].y);

                        for (int j = 0; j < pointList.size(); j++)
                        {
                            if (fabs(pointList.at(j).x - P.x) < 10 && fabs(pointList.at(j).y - P.y) < 10)
                            {
                                exsit = true;
                                break;
                            }
                        }
                        if (!exsit || pointList.size() == 0)
                            pointList.append(P); //pointList赋值    筛选后的三层同心中心点
                        if (i == 2)
                            OutquadList.append(contours_poly[i]); //OutquadList赋值    最外层外接四边形
                    }
                }

                //8    //最终筛选，保留可能性最大的三个角点和轮廓
                if (RectList.size() > 3)
                {
                    QList<float> RectSizeErrorList; //尺寸误差
                    for (int i = 0; i < RectList.size(); i++)
                    {
                        float RectSizeError = 0;
                        float RectSize1 = (RectList.at(i).size.width + RectList.at(i).size.height) * 2;
                        for (int j = 0; j < RectList.size(); j++ && j != i)
                        {
                            float RectSize2 = (RectList.at(j).size.width + RectList.at(j).size.height) * 2;
                            float Error = fabs(RectSize1 - RectSize2);
                            RectSizeError += Error;
                        }
                        RectSizeErrorList.append(RectSizeError);
                    }
                    QList<float> RectAngleErrorList; //角度误差
                    for (int i = 0; i < RectList.size(); i++)
                    {
                        float RectAngleError = 0;
                        float RectAngle1 = RectList.at(i).angle;
                        for (int j = 0; j < RectList.size(); j++ && j != i)
                        {
                            float RectAngle2 = RectList.at(j).angle;
                            float Error = fabs(RectAngle1 - RectAngle2);
                            RectAngleError += Error;
                        }
                        RectAngleErrorList.append(RectAngleError);
                    }
                    QList<float> RectErrorList; //综合误差
                    for (int i = 0; i < RectList.size(); i++)
                        RectErrorList.append(RectSizeErrorList.at(i) + RectAngleErrorList.at(i));
                    for (int i = RectErrorList.size() - 2; i >= 0; i--) //根据综合误差 对 矩形链表 进行排序(从小到大)
                        for (int j = 0; j <= i; j++)
                        {
                            if (RectErrorList.at(j + 1) < RectErrorList.at(j))
                            {
                                RectErrorList.swap(j + 1, j);
                                RectList.swap(j + 1, j);
                                pointList.swap(j + 1, j);
                                OutquadList.swap(j + 1, j);
                            }
                        }
                    //剔除误识别点
                    while (RectList.size() > 3)
                        RectList.removeLast();
                    while (pointList.size() > 3)
                        pointList.removeLast();
                    while (OutquadList.size() > 3)
                        OutquadList.removeLast();
                }
                else if (RectList.size() < 3)
                {
                    std::string text = "*";
                    // available=false;
                    int font_face = cv::FONT_HERSHEY_COMPLEX;
                    Point origin;
                    double font_scale = 1;
                    int thickness = 2;
                    origin.x = 50;
                    origin.y = 40;
                    cv::putText(image, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
                }
                if (pointList.size() == 3)
                {
                    QList<float> angleList;
                    for (int i = 0; i < pointList.size(); i++) //计算每个点的内角
                    {
                        float angle = 0;
                        Point thispoint = pointList.at(i); //本点
                        Point otherpoint[2];               //其余两个点
                        if (i == 0)
                        {
                            otherpoint[0] = pointList.at(1);
                            otherpoint[1] = pointList.at(2);
                        }
                        else if (i == 1)
                        {
                            otherpoint[0] = pointList.at(0);
                            otherpoint[1] = pointList.at(2);
                        }
                        else
                        {
                            otherpoint[0] = pointList.at(0);
                            otherpoint[1] = pointList.at(1);
                        }
                        float a = sqrt(pow(thispoint.x - otherpoint[1].x, 2) +
                                       pow(thispoint.y - otherpoint[1].y, 2)); //边a（otherpoint[0]的对边）
                        float b = sqrt(pow(otherpoint[0].x - otherpoint[1].x, 2) +
                                       pow(otherpoint[0].y - otherpoint[1].y, 2)); //边b（thispoint的对边）
                        float c = sqrt(pow(thispoint.x - otherpoint[0].x, 2) +
                                       pow(thispoint.y - otherpoint[0].y, 2)); //边c（otherpoint[1]的对边）
                        angle = acos((a * a + c * c - b * b) / (2 * a * c)) * 180 / M_PI;
                        angleList.append(angle);
                    }

                    for (int i = angleList.size() - 2; i >= 0; i--) //确定0号点位置
                        for (int j = 0; j <= i; j++)
                        {
                            float error1 = fabs(angleList.at(j) - 90);
                            float error2 = fabs(angleList.at(j + 1) - 90);
                            if (error2 < error1)
                            {
                                angleList.swap(j + 1, j);
                                pointList.swap(j + 1, j);
                                RectList.swap(j + 1, j);
                                OutquadList.swap(j + 1, j);
                            }
                        }
                    float Angle = getAngelOfTwoVector(pointList.at(1), pointList.at(2), pointList.at(0)); //以0为中心，2到1的角度
                    if (Angle < 0)                                                                        //确定1,2号点位置
                    {
                        pointList.swap(1, 2);
                        OutquadList.swap(1, 2);
                    }

                    //粗略计算QR码中心
                    Point2f P0;
                    P0.x = (pointList.at(1).x + pointList.at(2).x) / 2;
                    P0.y = (pointList.at(1).y + pointList.at(2).y) / 2;

                    //取出OutquadList的4*3个角点 到 cornerPointList
                    vector<Point> cornerPointList;
                    for (int i = 0; i < OutquadList.size(); i++)
                    {
                        vector<Point> points(OutquadList.at(i).size());
                        points = OutquadList.at(i);
                        for (int j = 0; j < points.size(); j++)
                            cornerPointList.push_back(points[j]);
                    }
                    //

                    //
                    // 4 个角点排序 function
                    Point image_cen;
                    Point A_cen;
                    image_cen.x = (pointList[1].x + pointList[2].x) / float(2);
                    image_cen.y = (pointList[1].y + pointList[2].y) / float(2);
                    A_cen.x = pointList[0].x;
                    A_cen.y = pointList[0].y;
                    QList<Point> sortList_B;
                    QList<Point> OutquadList_B;
                    QList<Point> sortList_C;
                    QList<Point> OutquadList_C;
                    int num_B = 1;
                    int num_C = 2;
                    OutquadList_B = Qlist_vector2Qlist(OutquadList_B, OutquadList, num_B);
                    sortList_B = sortPoint_B(OutquadList_B, sortList_B, image_cen, A_cen);
                    OutquadList_C = Qlist_vector2Qlist(OutquadList_C, OutquadList, num_C);
                    sortList_C = sortPoint_C(OutquadList_C, sortList_C, image_cen, A_cen);
                    // cout<<"------------"<<endl;
                    // std::cout<<image_cen.x<<std::endl;
                    // std::cout<<image_cen.y<<std::endl;
                    // std::cout<<A_cen.x<<std::endl;
                    // std::cout<<A_cen.y<<std::endl;

                    // std::cout<<sortList_B[0]<<std::endl;
                    // std::cout<<sortList_B[1]<<std::endl;
                    // std::cout<<sortList_B[2]<<std::endl;
                    // std::cout<<sortList_B[3]<<std::endl;
                    // std::cout<<sortList_C[0]<<std::endl;
                    // std::cout<<sortList_C[1]<<std::endl;
                    // std::cout<<sortList_C[2]<<std::endl;
                    // std::cout<<sortList_C[3]<<std::endl;
                    // cout<<"************"<<endl;

                    //test function

                    // cout<<"------------"<<endl;

                    // cout<<OutquadList[1][0]<<endl;
                    // // OutquadList.removeAt(0);
                    // cout<<OutquadList.size()<<endl;
                    // cout<<OutquadList[1]<<endl;
                    // cout<<OutquadList[2]<<endl;
                    // cout<<"************"<<endl;
                    // cout<<pointList[0]<<endl;
                    // cout<<pointList[1]<<endl;
                    // cout<<pointList[2]<<endl;
                    // cout<<"*************"<<endl;
                    Point P1_0, P1_1, P1_2, P1_3, P2_0, P2_1, P2_2, P2_3;

                    //求解另外四个点
                    P1_0 = sortList_B[0];
                    P1_2 = sortList_B[2];
                    P1_1 = sortList_B[1];
                    P1_3 = sortList_B[3];
                    P2_0 = sortList_C[0];
                    P2_1 = sortList_C[1];
                    P2_2 = sortList_C[2];
                    P2_3 = sortList_C[3];

                    Point P3_0 = CrossPoint(P1_0, P1_3, P2_0, P2_1);

                    // cout << "P3_0 " << P3_0.x << ";;;" << P3_0.y << endl;
                    Point P3_1 = CrossPoint(P1_1, P1_2, P2_0, P2_1);
                    // cout << "P3_1 " << P3_1.x << ";;;" << P3_1.y << endl;
                    Point P3_2 = CrossPoint(P1_0, P1_3, P2_3, P2_2);
                    // cout << "P3_2 " << P3_2.x << ";;;" << P3_2.y << endl;
                    Point P3_3 = CrossPoint(P1_1, P1_2, P2_3, P2_2);
                    // cout << "P3_3 " << P3_3.x << ";;;" << P3_3.y << endl;
                    jump_front = jump_front + 1;
                  
                    if (jump_front > 10)
                    {
                        //计算4个中心点,A为起点，B为右上方，C为左下
                        x_A = (OutquadList[0][0].x + OutquadList[0][1].x + OutquadList[0][2].x + OutquadList[0][3].x) / float(4);
                        y_A = (OutquadList[0][0].y + OutquadList[0][1].y + OutquadList[0][2].y + OutquadList[0][3].y) / float(4);

                        x_B = (OutquadList[1][0].x + OutquadList[1][1].x + OutquadList[1][2].x + OutquadList[1][3].x) / float(4);
                        y_B = (OutquadList[1][0].y + OutquadList[1][1].y + OutquadList[1][2].y + OutquadList[1][3].y) / float(4);

                        x_C = (OutquadList[2][0].x + OutquadList[2][1].x + OutquadList[2][2].x + OutquadList[2][3].x) / float(4);
                        y_C = (OutquadList[2][0].y + OutquadList[2][1].y + OutquadList[2][2].y + OutquadList[2][3].y) / float(4);

                        x_D = (P3_0.x + P3_1.x + P3_2.x + P3_3.x) / float(4);
                        y_D = (P3_0.y + P3_1.y + P3_2.y + P3_3.y) / float(4);
                        // cout<<"add_x:"<<P3_0.x + P3_1.x + P3_2.x + P3_3.x<<endl;
                        // cout<<"add_y:"<<P3_0.y + P3_1.y + P3_2.y + P3_3.y<<endl;

                        // cout << "x_A " << x_A << "  Y_A " << y_A << endl;
                        // cout << "x_B " << x_B << " y_B " << y_B << endl;
                        // cout << "x_C " << x_C << " y_C " << y_C << endl;
                        // cout << "x_D " << x_D << " y_D" << y_D << endl;
                        // cout<<"******"<<endl;
                        std::vector<std::vector<cv::Point2f>> corners;
                        std::vector<cv::Point2f> corner;
                        corner.push_back(cv::Point2f(x_A, y_A));
                        corner.push_back(cv::Point2f(x_B, y_B));
                        corner.push_back(cv::Point2f(x_D, y_D));
                        corner.push_back(cv::Point2f(x_C, y_C));

                        //

                        corners.push_back(corner);
                        // cout<<corners[0]<<endl;

                        std::vector<cv::Vec3d> rvecs, tvecs; // 储存结果R/t
                        // cv::aruco::estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs, rvecs, tvecs); // 求解旋转矩阵rvecs和平移矩阵tvecs
                        EstimatePoseSingleMarkers(corners,marker_size , cameraMatrix, distCoeffs, rvecs, tvecs);
                        // cout << "  T :" << tvecs[0] << endl;
                        // cout << "  R :" << rvecs[0] << endl;

                        // cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.5);
                        std::vector<int> ids;
                        ids.push_back(0);

                        //机械臂
                        // cout << "机械臂";
                        tvecs_sum[0] = tvecs_sum[0] + tvecs[0][0];
                        tvecs_sum[1] = tvecs_sum[1] + tvecs[0][1];
                        tvecs_sum[2] = tvecs_sum[2] + tvecs[0][2];
                        Mat R_1;
                        Rodrigues(rvecs, R_1);
                        Matrix<double, 3, 3> R_matrix;
                        cv2eigen(R_1, R_matrix);
                        Quaterniond quaternion(R_matrix);

                        // cout<<"四元数： "<<quaternion.x()<<", "<<quaternion.y()<<", "<<quaternion.z()<<", "<<quaternion.w()<<endl;

                        quat_sum[0] = quat_sum[0] + quaternion.x();
                        quat_sum[1] = quat_sum[1] + quaternion.y();
                        quat_sum[2] = quat_sum[2] + quaternion.y();
                        quat_sum[3] = quat_sum[3] + quaternion.w();

                        j = j + 1;
                        float sum{201};
                        float sum_mul = (sum - 1)*10000;
                        if (j == sum)
                        {

                            Mat R;
                            Mat pose_new_p = (Mat_<double>(1, 3) << tvecs_sum[0] / sum_mul, tvecs_sum[1] / sum_mul, tvecs_sum[2] / sum_mul);
                            // cout << "T   " << pose_new_p * 1000 << endl;
                            // cout<<"  R :"<<rvecs[0]<<endl;
                            Quaterniond quaternion_mean;
                            quaternion_mean.x() = quat_sum[0] / (sum - 1);
                            quaternion_mean.y() = quat_sum[1] / (sum - 1);
                            quaternion_mean.z() = quat_sum[2] / (sum - 1);
                            quaternion_mean.w() = quat_sum[3] / (sum - 1);
                            Matrix<double, 3, 3> r_matrix;
                            r_matrix = quaternion_mean.matrix();
                            eigen2cv(r_matrix, R);
                            // Rodrigues(rvecs, R);
                            Mat mark2cammera_new;
                            mark2cammera_new = rot_v2T(R, pose_new_p);
                            //中间矩阵计算
                            Mat cammer2eelink = Mat::eye(4, 4, CV_64FC1);
                            double distance = 0.3;
                            Mat mark2cammer = (Mat_<double>(4, 4) << 1, 0, 0, 0,
                                               0, -1, 0, 0,
                                               0, 0, -1, distance,
                                               0, 0, 0, 1);

                            Mat eelink2baselink;
                            vector<double> position = rtde_receive.getActualTCPPose();

                            Mat pose_P = (Mat_<double>(1, 3) << position[0], position[1], position[2]);
                            Mat pose_quat = (Mat_<double>(1, 3) << position[3], position[4], position[5]);
                            Mat pose_R;
                            Rodrigues(pose_quat, pose_R);
                            eelink2baselink = rot_v2T(pose_R, pose_P);
                            //
                            Mat eelink2baselink_new;
                            Mat mark2cammer_inv = mark2cammer.inv();
                            Mat cammer2eelink_inv = cammer2eelink.inv();
                            // cout<<"c2eelink_inv"<<cammer2eelink_inv<<endl;
                            eelink2baselink_new = eelink2baselink * (cammer2eelink * (mark2cammera_new * (mark2cammer_inv * (cammer2eelink_inv))));

                            //
                            Mat command_R = (Mat_<double>(3, 3) << eelink2baselink_new.at<double>(0, 0), eelink2baselink_new.at<double>(0, 1), eelink2baselink_new.at<double>(0, 2),
                                             eelink2baselink_new.at<double>(1, 0), eelink2baselink_new.at<double>(1, 1), eelink2baselink_new.at<double>(1, 2),
                                             eelink2baselink_new.at<double>(2, 0), eelink2baselink_new.at<double>(2, 1), eelink2baselink_new.at<double>(2, 2));
                            vector<double> RotationVector;
                            Rodrigues(command_R, RotationVector);
                            vector<double> send_command(6, 0);
                            vector<double> send_command_rqt(6, 0);
                            send_command[0] = eelink2baselink_new.at<double>(0, 3);
                            send_command[1] = eelink2baselink_new.at<double>(1, 3);
                            send_command[2] = eelink2baselink_new.at<double>(2, 3);
                            send_command[3] = RotationVector[0];
                            send_command[4] = RotationVector[1];
                            send_command[5] = RotationVector[2];

                            cout << "机械臂下发指令:";
                            for (int i = 0; i <= 5; i++)
                            {
                                // cout << "cammer2eelink" << position[ i ] << endl;
                                if (i < 3)
                                {
                                    send_command_rqt[i] = send_command[i] * 1000;
                                    cout << send_command[i] * 1000 << ",";
                                }
                                else
                                {
                                    send_command_rqt[i] = send_command[i];
                                    cout << send_command[i] << ",";
                                }
                            }
                            cout << endl;
                            int ret;
                            // ret = rtde_control.moveL(send_command, 0.1, 0.1);
                            // // cout << "run" << ret << endl;
                            jump_front = 0;
                            // cout << "  T :" << tvecs[0] << endl;
                            // cout << "  R :" << rvecs[0] << endl;

                            j = 1;
                            tvecs_sum[0] = 0.0;
                            tvecs_sum[1] = 0.0;
                            tvecs_sum[2] = 0.0;
                            quat_sum[0] = 0.0;
                            quat_sum[1] = 0.0;
                            quat_sum[2] = 0.0;
                            quat_sum[3] = 0.0;
                         }
                        // 机械臂
                        
                        DrawDetectedMarkers(image, corners, ids, Scalar(100, 0, 255));
                        DrawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 50, 3);

                        imshow("threshold_output", image);
                        waitKey(1);
                        sleep(0.1);
                    }
                }
            }
             
       //
     }
    }
    catch (exception &e)
    {
        cout << e.what() << '\n';
    }

    //     QList<Point> pointList; //存储角点中心
    //     QList<RotatedRect> RectList; //存储角元素最外层矩形
    //     QList<vector<Point>> OutquadList; //存储最外层拟合四边形角点
    //     vector<bool> qrPointListEnable(qrPointList.size()); //筛选时使用
    //     for (int L=0;L<qrPointList.size(); L++)//遍历每个可能的图元
    //     {
    //         qrPoint=qrPointList.at(L);
    //         vector<vector<Point>> contours_poly(qrPoint.size());
    //         vector<RotatedRect> minRect(qrPoint.size()); //存储了嵌套的最小外接矩形*****
    //         vector<Point2f> rect_center(qrPoint.size());
    //         for (int i = 0; i < qrPoint.size(); i++){
    //             approxPolyDP(Mat(qrPoint[i]), contours_poly[i], 5, true);//用指定精度逼近多边形曲线
    //             minRect[i] = minAreaRect(Mat(qrPoint[i])); //得到最小外接矩形
    //             rect_center[i]=minRect[i].center; //得到最小外接矩形中心
    //         }
    //         //根据同心度筛选
    //         for (int i = 0; i < minRect.size()-1; i++){
    //             Point P1=Point(rect_center[i].x,rect_center[i].y);
    //             Point P2=Point(rect_center[i+1].x,rect_center[i+1].y);
    //             float ConcenError_Set=(minRect[i].size.width+minRect[i].size.height)/12; //***最大允差设定***
    //             if( sqrt(pow(P1.x-P2.x,2)+pow(P1.y-P2.y,2)) > ConcenError_Set  ){
    //                 qrPointListEnable[L]=false;
    //                 break; }
    //             else
    //                 qrPointListEnable[L]=true;
    //         }
    //         if(!qrPointListEnable[L])continue;

    //         //根据三层周长比例进行筛选（3:5:7）
    //         for (int i = 0; i < minRect.size()-1; i++) {
    //             float circum1=(minRect[i].size.width+minRect[i].size.height)*2;
    //             float circum2=(minRect[i+1].size.width+minRect[i+1].size.height)*2;
    //             if( circum1/circum2>=0.5 && circum1/circum2<=0.8 )  //***周长比例设定***
    //                 qrPointListEnable[L]=true;
    //             else{
    //                 qrPointListEnable[L]=false;
    //                 break; }
    //         }
    //         if(!qrPointListEnable[L])continue;

    //         //周长不能过小
    //         for (int i = 0; i < minRect.size(); i++){
    //             float circum=(minRect[i].size.width+minRect[i].size.height)*2;
    //             float circum_Set=20;  //***有效周长最小值设定***
    //             if( circum >= circum_Set )
    //                 qrPointListEnable[L]=true;
    //             else{
    //                 qrPointListEnable[L]=false;
    //                 break; }
    //         }
    //         if(!qrPointListEnable[L])continue;

    //         //筛选完毕！！！筛选出的个数可能为任意自然数
    //         cout<<qrPoint.size()<<endl;
    //         for (int i = 0; i<qrPoint.size(); i++){
    //             Point2f rect_points[4];
    //             minRect[i].points(rect_points);
    //             if(i==2)
    //                 RectList.push_back(minRect[i]);        //RectList赋值   筛选后的最外层外接矩形
    //             bool exsit=false;
    //             Point P=Point(rect_center[i].x,rect_center[i].y);

    //             for(int j=0;j<pointList.size();j++){
    //                 if( fabs(pointList.at(j).x-P.x)<10 && fabs(pointList.at(j).y-P.y)<10 ){
    //                     exsit=true; break; }
    //             }
    //             if(!exsit||pointList.size()==0)
    //                 pointList.append(P);                   //pointList赋值    筛选后的三层同心中心点
    //             if(i==2)
    //                 OutquadList.append(contours_poly[i]);  //OutquadList赋值    最外层外接四边形
    //         }
    //     }

    // //8    //最终筛选，保留可能性最大的三个角点和轮廓
    //     if(RectList.size()>3){
    //         QList<float> RectSizeErrorList; //尺寸误差
    //         for(int i=0;i<RectList.size();i++){
    //             float RectSizeError=0;
    //             float RectSize1=( RectList.at(i).size.width + RectList.at(i).size.height )*2;
    //             for(int j=0;j<RectList.size();j++ && j!=i){
    //                 float RectSize2=( RectList.at(j).size.width + RectList.at(j).size.height )*2;
    //                 float Error= fabs( RectSize1 - RectSize2 );
    //                 RectSizeError+=Error;
    //             }
    //             RectSizeErrorList.append(RectSizeError);
    //         }
    //         QList<float> RectAngleErrorList; //角度误差
    //         for(int i=0;i<RectList.size();i++){
    //             float RectAngleError=0;
    //             float RectAngle1=RectList.at(i).angle;
    //             for(int j=0;j<RectList.size();j++ && j!=i){
    //                 float RectAngle2=RectList.at(j).angle;
    //                 float Error= fabs( RectAngle1 - RectAngle2 );
    //                 RectAngleError+=Error;
    //             }
    //             RectAngleErrorList.append(RectAngleError);
    //         }
    //         QList<float> RectErrorList; //综合误差
    //         for(int i=0;i<RectList.size();i++)
    //             RectErrorList.append(RectSizeErrorList.at(i)+RectAngleErrorList.at(i));
    //         for(int i=RectErrorList.size()-2;i>=0;i--) //根据综合误差 对 矩形链表 进行排序(从小到大)
    //             for(int j=0;j<=i;j++){
    //                 if(RectErrorList.at(j+1)<RectErrorList.at(j)){
    //                     RectErrorList.swap(j+1,j);
    //                     RectList.swap(j+1,j);
    //                     pointList.swap(j+1,j);
    //                     OutquadList.swap(j+1,j);
    //                 }
    //             }
    //         //剔除误识别点
    //         while(RectList.size()>3)   RectList.removeLast();
    //         while(pointList.size()>3)  pointList.removeLast();
    //         while(OutquadList.size()>3)  OutquadList.removeLast();
    //     }
    //     else if(RectList.size()<3)
    //     {
    //         std::string text = "*";
    //         // available=false;
    //         int font_face = cv::FONT_HERSHEY_COMPLEX;
    //         Point origin;
    //         double font_scale = 1;
    //         int thickness = 2;
    //         origin.x = 50; origin.y = 40;
    //         cv::putText(image, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);
    //     }
    //     cout<<pointList.size()<<endl;
    //  imshow("threshold_output",image);
    // waitKey(1);
    // // /*************************************重要代码段******************************************
    // // *至此已有     RectList：   type=QList<RotatedRect>     size=3 //存储角元素最外层最小外接矩形
    // // *            pointList：  type=QList<Point>           size=3 //存储角点中心
    // // *            OutquadList：type=QList<vector<Point>>   size=3 //存储最外层拟合四边形角点
    // // ***************************************************************************************/
    // //     //对角点和矩形进行位置排序（左上：1   右上：2   左下：3）
    //         cout<<pointList.size()<<endl;

    // QList<float> angleList;
    // for(int i=0;i<pointList.size();i++) //计算每个点的内角
    // {
    //     float angle=0;
    //     Point thispoint=pointList.at(i); //本点
    //     Point otherpoint[2]; //其余两个点
    //     if(i==0){
    //         otherpoint[0] = pointList.at(1);
    //         otherpoint[1] = pointList.at(2);}
    //     else if(i==1){
    //         otherpoint[0] = pointList.at(0);
    //         otherpoint[1] = pointList.at(2);}
    //     else{
    //         otherpoint[0] = pointList.at(0);
    //         otherpoint[1] = pointList.at(1);}
    //     float a=sqrt( pow(thispoint.x-otherpoint[1].x,2) + \
        //                   pow(thispoint.y-otherpoint[1].y,2) ); //边a（otherpoint[0]的对边）
    //     float b=sqrt( pow(otherpoint[0].x-otherpoint[1].x,2) + \
        //                   pow(otherpoint[0].y-otherpoint[1].y,2) ); //边b（thispoint的对边）
    //     float c=sqrt( pow(thispoint.x-otherpoint[0].x,2) + \
        //                   pow(thispoint.y-otherpoint[0].y,2) ); //边c（otherpoint[1]的对边）
    //     angle=acos( ( a*a + c*c -b*b ) / (2*a*c) )*180/M_PI;
    //     angleList.append(angle);
    // }
    // cout<<angleList.size()<<endl;
    // for(int i=angleList.size()-2;i>=0;i--) //确定0号点位置
    //     for(int j=0;j<=i;j++)
    //     {
    //         float error1=fabs(angleList.at(j)-90);
    //         float error2=fabs(angleList.at(j+1)-90);
    //         if(error2 < error1){
    //             angleList.swap(j+1,j);
    //             pointList.swap(j+1,j);
    //             RectList.swap(j+1,j);}
    //     }
    // float Angle=getAngelOfTwoVector(pointList.at(1),pointList.at(2),pointList.at(0)); //以0为中心，2到1的角度
    // if(Angle<0) //确定1,2号点位置
    //     pointList.swap(1,2);

    // //粗略计算QR码中心
    // Point2f P0;
    // P0.x = ( pointList.at(1).x + pointList.at(2).x ) / 2;
    // P0.y = ( pointList.at(1).y + pointList.at(2).y ) / 2;

    // //取出OutquadList的4*3个角点 到 cornerPointList
    // vector<Point> cornerPointList;
    // for(int i=0;i<OutquadList.size();i++){
    //     vector<Point> points(OutquadList.at(i).size());
    //     points=OutquadList.at(i);
    //     for(int j=0;j<points.size();j++)
    //         cornerPointList.push_back(points[j]);
    // }
    // // cout<<pointList[0]<<endl;
    // // cout<<pointList.at(0).y<<endl;
    // // cout<<pointList[1]<<endl;
    // // cout<<pointList[2]<<endl;

    // std::vector<std::vector<cv::Point2f>> corners;
    // std::vector<cv::Point2f> corner;
    // float x_4=pointList.at(1).x-pointList.at(0).x+pointList.at(2).x;
    // float y_4=pointList.at(1).y-pointList.at(0).y+pointList.at(2).y;
    // corner.push_back(cv::Point2f(pointList.at(0).x,pointList.at(0).y ));
    // corner.push_back(cv::Point2f(pointList.at(1).x,pointList.at(1).y ));
    // corner.push_back(cv::Point2f(pointList.at(2).x,pointList.at(2).y ));
    // corner.push_back(cv::Point2f(x_4,y_4));
    // corners.push_back(corner);
    // cout<<corners[0]<<endl;

    // std::vector<cv::Vec3d> rvecs, tvecs;                     // 储存结果R/t
    // cv::aruco::estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs, rvecs, tvecs); // 求解旋转矩阵rvecs和平移矩阵tvecs
    // // EstimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);

    // cout<<rvecs[0]<<endl;
    // cout<<tvecs[0]<<endl;

    // }

    //针对cornerPointList的防抖算法
    //antiShake(cornerPointList);

    //按一定规则对这12个点重新排序
    // sortNeartofar(cornerPointList,0,12,pointList.at(0));
    // sortNeartofar(cornerPointList,4,12,pointList.at(1));
    // vector<Point> cornerPointList_0;
    // vector<Point> cornerPointList_1;
    // vector<Point> cornerPointList_2;
    // for(int i=0;i<4;i++){
    //     cornerPointList_0.push_back(cornerPointList[i]);
    //     cornerPointList_1.push_back(cornerPointList[i+4]);
    //     cornerPointList_2.push_back(cornerPointList[i+8]);
    // }
    // Point P0_0=getFarestPoint(cornerPointList_0,P0);
    // Point P0_3=getNearestPoint(cornerPointList_0,P0);
    // Point P0_2=getFarestPoint(cornerPointList_0,pointList.at(1));
    // Point P0_1=getNearestPoint(cornerPointList_0,pointList.at(1));
    // Point P1_1=getFarestPoint(cornerPointList_1,P0);
    // Point P1_2=getNearestPoint(cornerPointList_1,P0);
    // Point P1_3=getFarestPoint(cornerPointList_1,pointList.at(0));
    // Point P1_0=getNearestPoint(cornerPointList_1,pointList.at(0));
    // Point P2_2=getFarestPoint(cornerPointList_2,P0);
    // Point P2_1=getNearestPoint(cornerPointList_2,P0);
    // Point P2_3=getFarestPoint(cornerPointList_2,pointList.at(0));
    // Point P2_0=getNearestPoint(cornerPointList_2,pointList.at(0));
    // Point P3_0=CrossPoint(P1_0,P1_2,P2_0,P2_1);
    // Point P3_1=CrossPoint(P1_1,P1_3,P2_0,P2_1);
    // Point P3_2=CrossPoint(P1_0,P1_2,P2_2,P2_3);
    // Point P3_3=CrossPoint(P1_1,P1_3,P2_2,P2_3);
    // circle(srcImgF, P0_0, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P0_1, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P0_2, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P0_3, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P1_0, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P1_1, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P1_2, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P1_3, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P2_0, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P2_1, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P2_2, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P2_3, 4,Scalar(0,255,255),4,1);
    // circle(srcImgF, P3_0, 2,Scalar(0,255,255),4,1);
    // circle(srcImgF, P3_1, 2,Scalar(0,255,255),4,1);
    // circle(srcImgF, P3_2, 2,Scalar(0,255,255),4,1);
    // circle(srcImgF, P3_3, 2,Scalar(0,255,255),4,1);

    // //计算4个中心点
    // Point2f P0_C,P1_C,P2_C,P3_C;
    // P0_C.x=float(P0_0.x+P0_1.x+P0_2.x+P0_3.x)/float(4);
    // P0_C.y=float(P0_0.y+P0_1.y+P0_2.y+P0_3.y)/float(4);
    // P1_C.x=float(P1_0.x+P1_1.x+P1_2.x+P1_3.x)/float(4);
    // P1_C.y=float(P1_0.y+P1_1.y+P1_2.y+P1_3.y)/float(4);
    // P2_C.x=float(P2_0.x+P2_1.x+P2_2.x+P2_3.x)/float(4);
    // P2_C.y=float(P2_0.y+P2_1.y+P2_2.y+P2_3.y)/float(4);
    // P3_C.x=float(P3_0.x+P3_1.x+P3_2.x+P3_3.x)/float(4);
    // P3_C.y=float(P3_0.y+P3_1.y+P3_2.y+P3_3.y)/float(4);

    // //重新赋值pointLists, size变化：size=3 -> size=4
    // QList<Point2f> poin2ftList;
    // poin2ftList.clear();
    // poin2ftList.append(P0_C);
    // poin2ftList.append(P1_C);
    // poin2ftList.append(P2_C);
    // poin2ftList.append(P3_C);

    // //重新计算中点
    // P0.x = ( poin2ftList.at(0).x + poin2ftList.at(1).x\
        //        + poin2ftList.at(2).x + poin2ftList.at(3).x) / 4;
    // P0.y = ( poin2ftList.at(0).y + poin2ftList.at(1).y\
        //        + poin2ftList.at(2).y + poin2ftList.at(3).y) / 4;

    // //绘制三角形连线
    // line(srcImgF,poin2ftList.at(0),poin2ftList.at(1),Scalar(0,255,255),1);
    // line(srcImgF,poin2ftList.at(1),poin2ftList.at(2),Scalar(0,255,255),2);
    // line(srcImgF,poin2ftList.at(0),poin2ftList.at(2),Scalar(0,255,255),1);
    // line(srcImgF,poin2ftList.at(0),poin2ftList.at(3),Scalar(0,255,255),2);
    // line(srcImgF,poin2ftList.at(1),poin2ftList.at(3),Scalar(0,255,255),1);
    // line(srcImgF,poin2ftList.at(2),poin2ftList.at(3),Scalar(0,255,255),1);

    // //计算屏幕中心点
    // Point2f PScreen0;
    // PScreen0.x = float(cols) / float(2);
    // PScreen0.y = float(rows) / float(2);

    // //绘制二维码正方向箭头
    // Point2f P0_C_1_C;
    // P0_C_1_C.x = ( poin2ftList.at(0).x + poin2ftList.at(1).x ) / float(2);
    // P0_C_1_C.y = ( poin2ftList.at(0).y + poin2ftList.at(1).y ) / float(2);
    // Point2f PFront;
    // PFront.x = ( P0_C_1_C.x + P0_C_1_C.x-P0.x );
    // PFront.y = ( P0_C_1_C.y + P0_C_1_C.y-P0.y );
    // drawArrow( srcImgF, P0, PFront, 17, 15, Scalar(0,255,0), 4, 4);

    // Point2f PX; //X轴正方向
    // PX.x = P0.x+10;
    // PX.y = P0.y;

    // float side01=sqrt( pow(P0_0.x-P1_1.x,2) + pow(P0_0.y-P1_1.y,2) ); //边01
    // float side12=sqrt( pow(P1_1.x-P2_2.x,2) + pow(P1_1.y-P2_2.y,2) ); //边12
    // float side23=sqrt( pow(P2_2.x-P3_3.x,2) + pow(P2_2.y-P3_3.y,2) ); //边23
    // float side30=sqrt( pow(P3_3.x-P0_0.x,2) + pow(P3_3.y-P0_0.y,2) ); //边30
    // float QRMeatrue=QRrealSize*4/(side01+side12+side23+side30);

    // QRX=(P0.x-PScreen0.x)*QRMeatrue; //QR码在x方向偏差（像素）
    // QRY=(PScreen0.y-P0.y)*QRMeatrue; //QR码在y方向偏差（像素）
    // QRAngle=getAngelOfTwoVector(P0_C_1_C,PX,P0); //QR码正方向相对于X轴正方向的夹角

    // while (true)
    // {
    //     // cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.5);
    //     // DrawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 5, 3);

    // }
   
    return 0;
}
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include "marker.h"
#include <opencv2/imgproc/types_c.h>
#include <zbar.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

using namespace Eigen;
using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
using namespace zbar; //添加zbar名称空间
using namespace cv;
RTDEControlInterface rtde_control("192.168.3.101");
RTDEReceiveInterface rtde_receive("192.168.3.101");

inline Mat rot_v2T(const Mat R, const Mat P)
{
    Mat T = (Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), P.at<double>(0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), P.at<double>(1),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), P.at<double>(2),
             0, 0, 0, 1);
    return T;
}

class QR_code
{
private:
    int sum = 200; //类加次数
    double dist = 0.3;
    cv::Vec3d tvec_sum;
    int count = 0;
    double marker_size=0.13;
    Mat R_1;
    Matrix<double, 3, 3> R_matrix;
    //定义4个元素，初始为0
    //内参 相机标定的参数-------
    // double fx = 916.22045898;
    // double cx = 648.51403809;
    // double fy = 914.37493896;
    // double cy = 379.04403687;
    double fx = 1364.61;
    double cx = 972;
    double fy = 1361.76;
    double cy = 556.603;

    double k1 = 0;
    double k2 = 0.;
    double p1 = 0;
    double p2 = 0;
    double k3 = 0;
    std_msgs::Float64MultiArray msg;
    std_msgs::Float64MultiArray msg_charuco;
    //ros
    ros::NodeHandle n;
    ros::Publisher sendcommand_pub = n.advertise<std_msgs::Float64MultiArray>("sendcommand", 1);
    ros::Publisher charuco_pub = n.advertise<std_msgs::Float64MultiArray>("charuco", 1);

    //ros
public:
    void robot_move(cv::Vec3d &rvec, cv::Vec3d &tvec)
    {

        static vector<double> quat_sum(4, 0);
        // rot累加
        // cout << "rvec" << rvec << endl;
        tvec_sum[0] = tvec_sum[0] + tvec[0];
        tvec_sum[1] = tvec_sum[1] + tvec[1];
        tvec_sum[2] = tvec_sum[2] + tvec[2];
        //转四元数
        Rodrigues(rvec, R_1);    // rotation vector (3x1 or 1x3) or rotation matrix (3x3)
        cv2eigen(R_1, R_matrix); // cv和eigen矩阵互换
        // cout << "矩阵互换:cv矩阵:" << R_1 << "eigen矩阵" << R_matrix << endl;
        Quaterniond quaternion(R_matrix);
        // cout<<"四元数： "<<quaternion.x()<<", "<<quaternion.y()<<endl;
        quat_sum[0] = quat_sum[0] + quaternion.x();
        quat_sum[1] = quat_sum[1] + quaternion.y();
        quat_sum[2] = quat_sum[2] + quaternion.y();
        quat_sum[3] = quat_sum[3] + quaternion.w();
        count++;
        //** 机械臂运动 **//
        if (count == sum)
        {

            //** 求平均 **//
            Mat R;
            Mat pose_new_p = (Mat_<double>(1, 3) << tvec_sum[0] / sum, tvec_sum[1] / sum, tvec_sum[2] / sum);
            // cout << "识别二维码距离(mm): " << pose_new_p * 1000 << endl;
            Quaterniond quaternion_mean(0, 0, 0, 0);
            quaternion_mean.x() = quat_sum[0] / sum;
            quaternion_mean.y() = quat_sum[1] / sum;
            quaternion_mean.z() = quat_sum[2] / sum;
            quaternion_mean.w() = quat_sum[3] / sum;

            Matrix<double, 3, 3> r_matrix;
            r_matrix = quaternion_mean.matrix();
            eigen2cv(r_matrix, R);

            pose_new_p = rot_v2T(R, pose_new_p);

            // cout << "pose_new_p " << pose_new_p << endl;
            //矩阵计算
            Mat cammer2eelink = Mat::eye(4, 4, CV_64FC1);
            Mat mark2cammer = (Mat_<double>(4, 4) << 1, 0, 0, -0.1,
                               0, -1, 0, 0.10,
                               0, 0, -1, dist,
                               0, 0, 0, 1);

            vector<double> position = rtde_receive.getActualTCPPose();
            Mat pose_P = (Mat_<double>(1, 3) << position[0], position[1], position[2]);
            Mat pose_R = (Mat_<double>(1, 3) << position[3], position[4], position[5]);
            Rodrigues(pose_R, pose_R);
            Mat eelink2baselink = rot_v2T(pose_R, pose_P);

            Mat mark2cammer_inv = mark2cammer.inv();
            Mat cammer2eelink_inv = cammer2eelink.inv();

            Mat eelink2baselink_new = eelink2baselink * cammer2eelink * pose_new_p * mark2cammer_inv * cammer2eelink_inv;
            //机械臂下发命令

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

            std::vector<double> send_charuco(7, 0);
            send_charuco[0] = tvec_sum[0] / sum;
            send_charuco[1] = tvec_sum[1] / sum;
            send_charuco[2] = tvec_sum[2] / sum;
            send_charuco[3] = quat_sum[0];
            send_charuco[4] = quat_sum[1];
            send_charuco[5] = quat_sum[2];
            send_charuco[6] = quat_sum[3];

            //  std_msgs::Float64MultiArray msg;
            cout << "机械臂下发指令: ";
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

            //ros

            msg.data = send_command_rqt;
            msg_charuco.data = send_charuco;
            sendcommand_pub.publish(msg);
            charuco_pub.publish(msg_charuco);

            //ros

            rtde_control.moveL(send_command, 0.1, 0.1);
            //****
            count = 0;
            tvec_sum[0] = 0.0;
            tvec_sum[1] = 0.0;
            tvec_sum[2] = 0.0;
            quat_sum[0] = 0.0;
            quat_sum[1] = 0.0;
            quat_sum[2] = 0.0;
            quat_sum[3] = 0.0;
        }
    }

    void generatePose()
    {

        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0.0, cx,
                                0.0, fy, cy,
                                0.0, 0.0, 1.0);
        cv::Mat distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

        cv::VideoCapture inputVideo;
        inputVideo.open(6);
        inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        ImageScanner scanner;
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
        // readCameraParameters( cameraMatrix, distCoeffs );

        while (inputVideo.grab())
        {

           cv::Mat image;

        inputVideo >> image;
        Mat imageGray;
        cvtColor(image, imageGray, CV_RGB2GRAY);
        int width = imageGray.cols;
        int height = imageGray.rows;

        uchar *raw = (uchar *)imageGray.data;
        Image imageZbar(width, height, "Y800", raw, width * height);

        scanner.scan(imageZbar); //扫描QR码

        Image::SymbolIterator symbol = imageZbar.symbol_begin();
        if (imageZbar.symbol_begin() == imageZbar.symbol_end())
        {
            cout << "can't detect QR code！" << endl;
        }
        else
        {
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<cv::Vec3d> rvecs, tvecs;
            for (int i = 0; symbol != imageZbar.symbol_end(); ++symbol)
            {
                // cout<<"type："<<endl<<symbol->get_type_name()<<endl<<endl;
                // cout<<"data："<<endl<<symbol->get_data()<<endl<<endl;
                // cout<<"data_length："<<endl<<symbol->get_data_length()<<endl<<endl;
                // cout<<"location_size："<<endl<<symbol->get_location_size()<<endl<<endl;
                std::vector<cv::Point2f> corner;
                corner.push_back(cv::Point2f(symbol->get_location_x(0), symbol->get_location_y(0)));
                corner.push_back(cv::Point2f(symbol->get_location_x(3), symbol->get_location_y(3)));
                corner.push_back(cv::Point2f(symbol->get_location_x(2), symbol->get_location_y(2)));
                corner.push_back(cv::Point2f(symbol->get_location_x(1), symbol->get_location_y(1)));
                corners.push_back(corner);
                ids.push_back(i);
                i++;
            }
            myEstimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);
            namedWindow("out", 0); //创建窗口
            cv::imshow("out", image);
            cv::resizeWindow("out", 500, 500); //创建一个500*500大小的窗口
            waitKey(1);
        }

        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "charuco"); // 节点名称

    QR_code QR_code;
    // charuco.generatePose( );
    QR_code.generatePose();

    return 0;
}

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

Mat rot_v2T(Mat R, Mat P)
{
    Mat T = (Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), P.at<double>(0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), P.at<double>(1),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), P.at<double>(2),
             0, 0, 0, 1);

    return T;
}
int main(int argc, char *argv[])
{

    //
    ros::init(argc, argv, "talker"); // 节点名称
    ros::NodeHandle n;
    ros::Publisher sendcommand_pub = n.advertise<std_msgs::Float64MultiArray>("sendcommand", 1);

    //

    RTDEControlInterface rtde_control("192.168.3.101");
    RTDEReceiveInterface rtde_receive("192.168.3.101");
    // std::vector<double> joint_positions = rtde_receive.getActualQ();
    cout << "CV_VERSION: " << CV_VERSION << endl;
    //相机标定的参数-------
   double fx = 1364.61;
    double cx = 972;
    double fy = 1361.76;
    double cy = 556.603;

    double k1 = 0;
    double k2 = 0.;
    double p1 = 0;
    double p2 = 0;
    double k3 = 0;
    //相机内参
    Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0);

    //畸变矩阵
    Mat distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
    //相机标定的参数-------
    //二维码的边长
    double marker_size = 149; //单位  mm
    //zbar::ImageScanner
    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    //仅仅创建一个捕获对象，而不提供任何关于打开的信息。
    cv::VideoCapture inputVideo(6);
    //如果只有1个摄像机，那么就是0，如果系统中有多个摄像机，那么只要将其向上增加即可。6为realsense
    // inputVideo.open(6);

    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    int j = 1;
    vector<double> tvecs_sum(3, 0);
    vector<double> quat_sum(4, 0);

    while (inputVideo.grab())
    {
        cv::Mat image1;
        cv::Mat image_out;
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
            // cout << "corners" << corners[0] << endl;
            // cout<<endl;
            //求解旋转向量rvecs和平移向量tvecs,

            myEstimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);

            tvecs_sum[0] = tvecs_sum[0] + tvecs[0][0];
            tvecs_sum[1] = tvecs_sum[1] + tvecs[0][1];
            tvecs_sum[2] = tvecs_sum[2] + tvecs[0][2];
            Mat R_1;
            Rodrigues(rvecs, R_1);
            Matrix<double, 3, 3> R_matrix;
            cv2eigen(R_1, R_matrix);
            Quaterniond quaternion(R_matrix);
            quat_sum[0] = quat_sum[0] + quaternion.x();
            quat_sum[1] = quat_sum[1] + quaternion.y();
            quat_sum[2] = quat_sum[2] + quaternion.y();
            quat_sum[3] = quat_sum[3] + quaternion.w();

            j = j + 1;
            int sum{201};
            int sum_mul = (sum - 1) * 1000;
            if (j == sum)
            {

                // cout<<"tttt"<<tvecs_sum[0]/300<<endl;
                Mat R;
                Mat pose_new_p = (Mat_<double>(1, 3) << tvecs_sum[0] / sum_mul, tvecs_sum[1] / sum_mul, tvecs_sum[2] / sum_mul);
                cout << "pose_new_p" << pose_new_p * 1000 << endl;
                cout << endl;
                Quaterniond quaternion_mean;
                quaternion_mean.x() = quat_sum[0] / (sum - 1);
                quaternion_mean.y() = quat_sum[1] / (sum - 1);
                quaternion_mean.z() = quat_sum[2] / (sum - 1);
                quaternion_mean.w() = quat_sum[3] / (sum - 1);
                Matrix<double, 3, 3> r_matrix;
                r_matrix = quaternion_mean.matrix();
                eigen2cv(r_matrix, R);
                Rodrigues(rvecs, R);
                Mat mark2cammera_new;
                mark2cammera_new = rot_v2T(R, pose_new_p);
                //中间矩阵计算
                Mat cammer2eelink = Mat::eye(4, 4, CV_64FC1);
                double distance = 0.3;
                Mat mark2cammer = (Mat_<double>(4, 4) << 1, 0.0, 0.0, 0.0,
                                   0, -1, 0, 0.0,
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
                send_command[0] = eelink2baselink_new.at<double>(0, 3);
                send_command[1] = eelink2baselink_new.at<double>(1, 3);
                send_command[2] = eelink2baselink_new.at<double>(2, 3);
                send_command[3] = RotationVector[0];
                send_command[4] = RotationVector[1];
                send_command[5] = RotationVector[2];

                //ros
                std_msgs::Float64MultiArray msg;
                vector<double> send_command_rqt(6, 0);
                cout << "send_command: ";
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

                // send_command_rqt[0] = send_command[0] * 1000;
                // send_command_rqt[1] = send_command[1] * 1000;
                // send_command_rqt[2] = send_command[2] * 1000;
                // send_command_rqt[3] = send_command[3] * 1000;
                // send_command_rqt[4] = send_command[4] * 1000;
                // send_command_rqt[5] = send_command[5] * 1000;

                // // //ros
                // std_msgs::Float64MultiArray msg;
                // vector<double> send_command_rqt(6, 0);
                // send_command_rqt[0] = send_command[0] * 1000;
                // send_command_rqt[1] = send_command[1] * 1000;
                // send_command_rqt[2] = send_command[2] * 1000;
                int ret;
                ret = rtde_control.moveL(send_command, 0.1, 0.1);
                // cout << "run" << ret << endl;

                msg.data = send_command_rqt;
                sendcommand_pub.publish(msg);
                ros::spinOnce();

                //ros

                
                sleep(1);

                j = 1;
                tvecs_sum[0] = 0.0;
                tvecs_sum[1] = 0.0;
                tvecs_sum[2] = 0.0;
                quat_sum[0] = 0.0;
                quat_sum[1] = 0.0;
                quat_sum[2] = 0.0;
                quat_sum[3] = 0.0;
            }

            for (int i = 0; i < ids.size(); i++)
            {
                // cv::aruco::drawDetectedMarkers(image, corners, ids);//绘制检测到的靶标的框
                myDrawDetectedMarkers(imageGray, corners, ids, Scalar(100, 0, 255)); //绘制检测到的靶标的框
                // cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 5);
                myDrawFrameAxes(imageGray, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 5, 3);
                // drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 5, 3);//opencv 3.3.1 没有 4.2.1有
            }

            imshow("Source Image", imageGray);
            waitKey(1);
        }
    }

    return 0;
}

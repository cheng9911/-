#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
// opencv Aruco官方示例
// https://github.com/opencv/opencv_contrib/tree/4.x/modules/aruco/samples

using namespace std;
using namespace cv;
using namespace ur_rtde;
using namespace std::chrono;
using namespace Eigen;

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

class Charuco
{
private:
    int sum = 200; //类加次数
    double dist = 0.3;
    cv::Vec3d tvec_sum;
    int count = 0;

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
    void generateMarker()

    {
        std::string charuco_path = "charuco.jpg";
        cv::Mat boardImage;
        // 声明字典，字典名称表示了该字典的aruco标记数量和尺寸
        Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        // cv::Ptr< cv::aruco::CharucoBoard > board = cv::aruco::CharucoBoard::create( 3, 3, 0.18f, 0.12f, dictionary );
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(4, 4, 0.6f, 0.5f, dictionary);
        board->draw(cv::Size(840, 640), boardImage, 10, 1);

        imwrite(charuco_path, boardImage);
        imshow("board", boardImage);
        waitKey(0);
        destroyAllWindows();
    }

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
            Mat mark2cammer = (Mat_<double>(4, 4) << 1, 0, 0, 0.0,
                               0, -1, 0, -0.0,
               
                               0, 0, -1, dist,
                               0, 0, 0, 1);
           
           
            vector< double > position = rtde_receive.getActualTCPPose( );
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
        // readCameraParameters( cameraMatrix, distCoeffs );
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        // cv::Ptr< cv::aruco::CharucoBoard > board    = cv::aruco::CharucoBoard::create( 3, 3, 0.0605f, 0.0505f, dictionary );
        // cv::Ptr< cv::aruco::CharucoBoard > board = cv::aruco::CharucoBoard::create( 4, 4, 0.0455f, 0.038f, dictionary );
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(4, 4, 0.0385f, 0.0323f, dictionary);

        while (inputVideo.grab())
        {

            cv::Mat image, imageCopy;
            inputVideo.retrieve(image);
            image.copyTo(imageCopy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);
            // cout << "ids" << ids.size( ) << endl;
            // if at least one marker detected
            if (ids.size() == 8)

            {
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(corners, ids, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0)
                {
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
                    cv::Vec3d rvec, tvec;
                    // cout << charucoCorners.size( ) << endl;
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                    {
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                        robot_move(rvec, tvec);
                        // std::cout << "T:" << tvec << endl;
                    }
                }
            }
            namedWindow("out", 0); //创建窗口
            cv::imshow("out", imageCopy);
            cv::resizeWindow("out", 500, 500); //创建一个500*500大小的窗口
            waitKey(1);
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "charuco"); // 节点名称

    Charuco charuco;
    // charuco.generatePose( );
    charuco.generatePose();

    return 0;
}

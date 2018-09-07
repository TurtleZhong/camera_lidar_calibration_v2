#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>

//message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>


#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <fstream>

#include <../include/camera_laser_calibration/config.h>
#include <Eigen/Dense>


using namespace std;
using namespace cv;
using namespace message_filters;

laser_geometry::LaserProjection projector_;

cv::Mat K;
cv::Mat D;
cv::Mat Rcl;
cv::Mat tcl;

ifstream inFile;

image_transport::Publisher output_image;
int image_count = 0;

void callback(const cv_bridge::CvImage::ConstPtr &imgMsg,const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{

    /**
     * Test the sync results!
     */
    ROS_INFO_STREAM("image timestamp: " << imgMsg->header.stamp.toNSec() << " ns");
    ROS_INFO_STREAM("scan  timestamp: " << laserMsg->header.stamp.toNSec() << " ns");
    double diff = imgMsg->header.stamp.toSec() - laserMsg->header.stamp.toSec();
    if(diff < 0)
        diff = -1.0 * diff;
    ROS_INFO_STREAM("           diff: " << diff << " s" );
    Mat inImage;
    undistort(imgMsg->image, inImage, K, D);
    cvtColor(inImage, inImage, COLOR_GRAY2BGR);

    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*laserMsg, cloud);
    vector<Point2d> pts_uv;

    for (int i = 0; i < cloud.points.size(); ++i)
    {

        /// Reprojection
        Mat point_l(3,1,CV_64FC1);
        point_l.at<double>(0,0) = cloud.points[i].x;
        point_l.at<double>(1,0) = cloud.points[i].y;
        point_l.at<double>(2,0) = cloud.points[i].z;
        Mat point_c = Rcl * point_l + tcl;


        if(point_c.at<double>(2,0) <= 0.)
            continue;
        point_c.at<double>(0,0) /= point_c.at<double>(2,0);
        point_c.at<double>(1,0) /= point_c.at<double>(2,0);
        point_c.at<double>(2,0) = 1.0;

        Mat uv = K * point_c;
        Point2d pt_uv(uv.at<double>(0,0), uv.at<double>(1,0));
        pts_uv.push_back(pt_uv);

    }
    ///Draw points in images
    for (int j = 0; j < pts_uv.size(); ++j) {
        cv::circle(inImage, pts_uv[j], 1, Scalar(0,255,0), 1);
    }
    cv_bridge::CvImage debug_image(imgMsg->header,"bgr8", inImage);
    if(output_image.getNumSubscribers())
        output_image.publish(debug_image.toImageMsg());
}

int main(int argc, char *argv[])
{

    if(argv[1]== nullptr)
    {
        cout << "Please Check the launch file" << endl;
    }
    string config_path(argv[1]);
    cout << "Config.path = " << config_path << endl;
    Config::setParameterFile(config_path);

    ros::init(argc, argv, "camera_laser_calibration");
    ros::NodeHandle nh;

    string scan_topic, image_topic;
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("image_topic", image_topic);

    string calib_result_path;
    nh.param<string>("calib_result_path", calib_result_path, "");
    inFile.open(calib_result_path);

    double q_x,q_y,q_z,q_w,t_x,t_y,t_z;
    inFile >> q_x >> q_y >> q_z >> q_w >> t_x >> t_y >> t_z;

    Eigen::Quaterniond q_cl(q_w, q_x, q_y, q_z);
    Eigen::Matrix3d tmp = q_cl.toRotationMatrix();
    Rcl = (Mat_<double>(3,3) << tmp(0,0), tmp(0,1), tmp(0,2),
                                tmp(1,0), tmp(1,1), tmp(1,2),
                                tmp(2,0), tmp(2,1), tmp(2,2));
    tcl = (Mat_<double>(3, 1) << t_x, t_y, t_z);

    ROS_INFO_STREAM("\n" << Rcl);
    ROS_INFO_STREAM("\n" << tcl);


    /// Corner detect
    double fx = Config::get<double>("fx");
    double fy = Config::get<double>("fy");
    double cx = Config::get<double>("cx");
    double cy = Config::get<double>("cy");

    double k1 = Config::get<double>("k1");
    double k2 = Config::get<double>("k2");
    double p1 = Config::get<double>("p1");
    double p2 = Config::get<double>("p2");

    K = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    D = (Mat_<double>(5,1) << k1, k2, p1, p2, 0.0);
    cout << "K:\n" << K << endl;
    cout << "D:\n" << D << endl;
    ROS_INFO_STREAM("Rcl = \n" << Rcl);
    ROS_INFO_STREAM("tcl = \n" << tcl);
    ROS_INFO_STREAM("image_topic: " << image_topic);
    ROS_INFO_STREAM("scan_topic: " << scan_topic);

    image_transport::ImageTransport it(nh);

    output_image = it.advertise("debug_reprojection", 1);
    message_filters::Subscriber<cv_bridge::CvImage> imgSub(nh, image_topic, 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> laserSub(nh, scan_topic, 1);

//
    typedef message_filters::sync_policies::ApproximateTime<cv_bridge::CvImage, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), imgSub, laserSub);
//    TimeSynchronizer<cv_bridge::CvImage, sensor_msgs::LaserScan> sync(imgSub, laserSub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}

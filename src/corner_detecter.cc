/**
 * This program used for detect 1 corner (subpixel) in the image
 * Input: config/config.yaml imgs/image_lists.txt
 * Output: data/image_points.txt
 * Author: xinliangzhong@foxmail.com
 * Data: 2018.07.05
 * Note: Remember to undistort the image.
 */

#include <camera_laser_calibration/config.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;


cv::Mat org,img,tmp;
ofstream outFile;
Point2f keypoint;

void cornerDetect(const cv::Mat &img, const Point &left_up, const Point &right_down)
{
    /// Create Mask
    Mat image = img.clone();
    cvtColor(image, image, COLOR_BGR2GRAY);
    Mat mask(image.rows, image.cols, CV_8U, Scalar(0));
    Mat detect_roi = image.rowRange(left_up.y, right_down.y).colRange(left_up.x, right_down.x);
    detect_roi.copyTo(mask.rowRange(left_up.y, right_down.y).colRange(left_up.x, right_down.x));

    /// Shi-Tomasi
    vector<Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    int maxCorners = 1;

    goodFeaturesToTrack( image,
                         corners,
                         maxCorners,
                         qualityLevel,
                         minDistance,
                         mask,
                         blockSize,
                         useHarrisDetector,
                         k );

    
    /// subpixel
    Size winSize = Size( 3, 3 );
    Size zeroZone = Size( -1, -1 );
    TermCriteria criteria = TermCriteria(
            CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
            40, //maxCount=40
            0.001 );  //epsilon=0.001
    cornerSubPix( image, corners, winSize, zeroZone, criteria );

    cvtColor(image, image, COLOR_GRAY2BGR);

    circle( image, corners[0], 1, Scalar(0,0,255), -1, 8, 0 );
    imshow("corner", image);
    cout << "corners: " << corners[0].x << " " << corners[0].y << endl;
    keypoint = corners[0];
}


void on_mouse(int event,int x,int y,int flags,void *ustc)
{
    static Point pre_pt(-1,-1);
    static Point cur_pt(-1,-1);
    char temp[16];
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        org.copyTo(img);
        sprintf(temp,"(%d,%d)",x,y);
        pre_pt = Point(x,y);
        putText(img,temp,pre_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0,255),1,8);
        circle(img,pre_pt,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
        imshow("img",img);
    }
    else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
    {
        img.copyTo(tmp);
        sprintf(temp,"(%d,%d)",x,y);
        cur_pt = Point(x,y);
        putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
        imshow("img",tmp);
    }
    else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
    {
        img.copyTo(tmp);
        sprintf(temp,"(%d,%d)",x,y);
        cur_pt = Point(x,y);
        putText(tmp,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
        rectangle(tmp,pre_pt,cur_pt,Scalar(255,0,0),1,8,0);
        imshow("img",tmp);
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        org.copyTo(img);
        sprintf(temp,"(%d,%d)",x,y);
        cur_pt = Point(x,y);
        putText(img,temp,cur_pt,FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255));
        circle(img,pre_pt,2,Scalar(255,0,0),CV_FILLED,CV_AA,0);
        rectangle(img,pre_pt,cur_pt,Scalar(255,0,0),1,8,0);
        imshow("img",img);
        /// Corner detector
        cornerDetect(org, pre_pt, cur_pt);
    }
}


int main(int argc, char* argv[])
{
    if(argv[1]== nullptr)
    {
        cout << "Please Check the launch file" << endl;
    }
    string config_path(argv[1]);
    cout << "Config.path = " << config_path << endl;
    Config::setParameterFile(config_path);
    string work_dir = Config::get<string>("working.dir");
    outFile.open(work_dir + "data/image_points.txt");

    /// Load images
    string images_dir = Config::get<string>("images.dir");
    string image_path = images_dir + "/image_lists.txt";
    vector<string> image_paths;
    ifstream inFile;
    inFile.open(image_path);
    while (inFile.good())
    {
        string image_name;
        inFile >> image_name;
        string img_path = images_dir + "/" + image_name;
        image_paths.push_back(img_path);
        inFile.get();
    }

    /// Corner detect
    double fx = Config::get<double>("fx");
    double fy = Config::get<double>("fy");
    double cx = Config::get<double>("cx");
    double cy = Config::get<double>("cy");

    double k1 = Config::get<double>("k1");
    double k2 = Config::get<double>("k2");
    double p1 = Config::get<double>("p1");
    double p2 = Config::get<double>("p2");

    cv::Mat K = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    cv::Mat D = (Mat_<double>(5,1) << k1, k2, p1, p2, 0.0);
    cout << "K:\n" << K << endl;
    cout << "D:\n" << D << endl;

    cout << "We need to process " << image_paths.size() << " images" << endl;
    for (int i = 0; i < image_paths.size(); ++i)
    {

        cout << "Processing images: " << i << endl;
        /// TO DO: Undistort the image
        org = imread(image_paths[i]);

        cv::undistort(org,img,K,D);
        org = img.clone();
//        org.copyTo(img);
        org.copyTo(tmp);
        namedWindow("img");
        setMouseCallback("img",on_mouse,0);
        imshow("img",img);
        cv::waitKey(0);
        destroyWindow("corner");
        outFile << keypoint.x << " " << keypoint.y << endl;
        cout << "Save " << i << " keypoint suscessfully" << endl;
    }
    destroyAllWindows();
    outFile.close();
    cout << "Save the sequence suscessfully" << endl;
    cout << "The output file saved in: " << work_dir + "data/image_points.txt" << endl;

    return 0;

}


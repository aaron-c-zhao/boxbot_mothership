#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Quaternion.h>

#include "boxbot_cv/SetBackground.h"
#include "boxbot_cv/LocateStacks.h"

#include "boxbot_cv/json.hpp"

#define C_WIDTH    0.355
#define C_HEIGHT   0.472


const std::string SHAPE = "[[[1044, 826]], [[1043, 827]], [[1042, 827]], [[1041, 828]], [[1038, 828]], [[1037, 827]], [[1035, 829]], [[1028, 829]], [[1027, 828]], [[1026, 829]], [[1024, 829]], [[1023, 828]], [[1017, 828]], [[1015, 830]], [[1013, 830]], [[1012, 829]], [[1008, 829]], [[1007, 830]], [[1006, 829]], [[1004, 829]], [[1003, 830]], [[1002, 830]], [[1003, 830]], [[1004, 831]], [[1009, 831]], [[1010, 832]], [[1011, 832]], [[1012, 831]], [[1013, 831]], [[1014, 832]], [[1015, 832]], [[1016, 833]], [[1016, 834]], [[1019, 834]], [[1019, 832]], [[1020, 831]], [[1022, 831]], [[1023, 832]], [[1023, 833]], [[1025, 835]], [[1033, 835]], [[1034, 834]], [[1035, 834]], [[1037, 832]], [[1039, 832]], [[1041, 830]], [[1044, 830]], [[1043, 829]], [[1043, 827]], [[1044, 826]], [[1045, 826]]]";


using namespace cv;

static const std::string OPENCV_WINDOW = "Frame";

class ImageConverter
{
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    ImageConverter(ros::NodeHandle nh)
        : it_(nh)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/boxbot_cam/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        background_ = imread("background.jpg");
        isBackgroundSet_ = true; 
        if (background_.empty()) {
            ROS_WARN("No background image available.");
            isBackgroundSet_ = false;
        }
        namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr_;
        try
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        image_ = cv_ptr_->image;
        // Update GUI Window
        imshow(OPENCV_WINDOW, image_);
        waitKey(3);
    }

    
    void locate() {
                
    } 

    void filter() {
        if(!isBackgroundSet_){
            ROS_ERROR("[Boxbot CV] background not set yet!");
        }

        // smoothing the image
        Mat img;
        //bilateralFilter(image_, img, 9, 75, 75);
        GaussianBlur(image_, img, Size(5, 5), 0);
        absdiff(image_, background_, img);

        cvtColor(img, img, COLOR_RGB2GRAY);

        // morphlogical open operation to get rid of the noise from bk subtraction
        Mat morph_elem = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(img, img, MORPH_OPEN, morph_elem);

        // median blur to remove the slat and pepper noise 
        medianBlur(img, img, 5);

        threshold(img, img, 50, 255, THRESH_BINARY | THRESH_OTSU );

        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        ROS_INFO_STREAM("Find " << contours.size() << "contours" << " size " << contours[0].size());

        std::vector<std::vector<Point>> validContours; 
        for (std::size_t i = 0; i < contours.size(); i++) {
            auto area = contourArea(contours[i]);
            ROS_INFO_STREAM("Contour " << i << " size: " << area);
            if (area > 10000) {
                auto match = matchShapes(shape_, contours[i], CONTOURS_MATCH_I1, 1.0);
                ROS_INFO_STREAM("Shape match - contour " << i << " " << match);
                if (match < 7.0) {
                    validContours.push_back(contours[i]);
                }
            }
        }

        Scalar color(0, 255, 255);
        std::vector<RotatedRect> boxes;
        for (std::size_t i = 0; i < validContours.size(); i++) {
            auto box = minAreaRect(validContours[i]);
            boxes.push_back(box);
            Point2f vtx[4];
            box.points(vtx);
            for (std::size_t j = 0; j < 4; j++) {
                line(image_, vtx[j], vtx[(j + 1) % 4], color, 1);
            }
            circle(image_, box.center, 2, color, 2);
        }
        
        mapToBase(boxes);

        circle(image_, Point((int)image_.size().width / 2, (int)image_.size().height / 2), 2, Scalar(255, 0, 0), 2);
        imshow("filter", image_);
        waitKey(3); 
    }



    void mapToBase(std::vector<RotatedRect> &boxes) {
        double widthSum = 0.0;
        double heightSum = 0.0;
        double centerX = image_.size().width / 2;
        double centerY = image_.size().height / 2;
        
        for (std::size_t i = 0; i < boxes.size(); i++)  {
            auto boxSize = boxes[i].size;
            widthSum += boxSize.width;
            heightSum += boxSize.height;
        }
        double pixelSize = ((C_WIDTH + C_HEIGHT) * boxes.size()) / (widthSum + heightSum);
        
        for (std::size_t i = 0; i < boxes.size(); i++) {
            auto box = boxes[i];
            double boxX = box.center.x * pixelSize;
            double boxY = box.center.y * pixelSize;
            std::ostringstream stringStream;
            stringStream << "(" << boxX << ", " << boxY << ")" << " angle: " << box.angle;
            putText(image_, stringStream.str(), Point(box.center.x + 10, box.center.y), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0));
        }
                

    }


    bool setBackgroundImg(boxbot_cv::SetBackground::Request &res,
        boxbot_cv::SetBackground::Response &req) {
        bilateralFilter(image_, background_, 9, 75, 75);
        if (background_.empty()) {
            ROS_ERROR("Can not set background.");
            return false;
        }

        imwrite("background.jpg", background_);
        isBackgroundSet_ = true;
        return true;
    }

    bool locateStacks(boxbot_cv::LocateStacks::Request &res,
        boxbot_cv::LocateStacks::Response &req) {
            filter();
            return true;
        }

    void loadShape(const std::string &shape) {
        auto json = nlohmann::json::parse(shape);
        for (std::size_t i = 0; i < json.size(); i++) {
            Point p(json[i][0][0], json[i][0][1]);
            shape_.push_back(p);
        }
        ROS_INFO("[Boxbot CV] Loaded standard shape information.");
    }

private:
    Mat image_;
    Mat background_;
    bool isBackgroundSet_ = false;
    std::vector<Point> shape_;
    std::vector<geometry_msgs::Quaternion> position_;

};




int main(int argc, char **argv)
{

    //
    bool flag = false;

    ros::init(argc, argv, "boxbot_cv");
    ros::NodeHandle nh;

    ros::Rate r(5);
    ImageConverter ic(nh);

    ic.loadShape(SHAPE);
    ros::ServiceServer backgrounSrv = nh.advertiseService("cv_bak", &ImageConverter::setBackgroundImg, &ic);
    ros::ServiceServer locateSrv = nh.advertiseService("locate_stacks", &ImageConverter::locateStacks, &ic);
     
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

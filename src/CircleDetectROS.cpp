   #include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64MultiArray.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher circle_pub;
  std_msgs::Float64MultiArray circle_data;
  ros::Publisher rad_pub;
  std_msgs::Float64MultiArray radii;
  int processingScale;
 
 
public:
  ImageConverter()
    : it_(nh_)
  {
    int scale;
    if (nh_.getParam("/processingScale", scale))
    {
      processingScale = scale;
    }
    else
    {
      processingScale = 1;
    }
    std::cout << " Scale: " << processingScale << std::endl;
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    cv::namedWindow(WINDOW);
    circle_pub = nh_.advertise<std_msgs::Float64MultiArray>("circle_data", 10);
    circle_data.data.resize(12);
  }
  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat CircleImage;

    cv::resize(cv_ptr->image,CircleImage,cv::Size(cv_ptr->image.cols/processingScale,cv_ptr->image.rows/processingScale));

    cv::cvtColor( CircleImage, CircleImage, CV_BGR2GRAY );
    //cout << "3 " << endl;

    cv::GaussianBlur( CircleImage, CircleImage, cv::Size(9, 9), 2, 2 );
    cv::vector<cv::Vec3f> circles;
    cv::HoughCircles( CircleImage, circles, CV_HOUGH_GRADIENT, 1, CircleImage.rows/(8*processingScale), 200, 100, 0, 0 );


    /// Draw the circles detected
    for( int i = 0; i < (circles.size()); i++ )
    {
            // ROS stuff
            circle_data.data[3*i]=circles[i][0]*processingScale;
            circle_data.data[3*i+1]=circles[i][1]*processingScale;
            circle_data.data[3*i+2]=circles[i][2]*processingScale;
    }
    if (circles.size()==4)
    {
      std::cout << circle_data << std::endl;
      circle_pub.publish(circle_data);
    }
    
    /// Draw the circles detected
    for( int i = 0; i < (circles.size()); i++ )
    {
            cv::Point center(cvRound(circles[i][0]*processingScale), cvRound(circles[i][1]*processingScale));
            int radius = cvRound(circles[i][2]*processingScale);
            // circle center
            cv::circle( cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            cv::circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    //std::cout << "circles" << circles.size() << std::endl;


    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processor");
  ImageConverter ic;
  ros::spin();
  return 0;
} 

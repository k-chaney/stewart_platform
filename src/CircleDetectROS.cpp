#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64MultiArray.h"
# include <math.h>
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
  double majorToMinor;
 
 
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
    int mTm;
    if (nh_.getParam("/majorToMinor", mTm))
    {
      majorToMinor = mTm;
    }
    else
    {
      majorToMinor = 1.5;
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

  double Distance(double dX0, double dY0, double dX1, double dY1)
  {
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
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
//    cv::vector<cv::Vec3f> circles;
//    cv::imshow(WINDOW, CircleImage);
//    cv::HoughCircles( CircleImage, circles, CV_HOUGH_GRADIENT, 1, CircleImage.rows/(8*processingScale), 200, 100, 0, 0 );

    cv::vector<std::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::Canny(CircleImage,CircleImage,100,300,3);
    //cv::imshow(WINDOW, CircleImage);
    cv::findContours(CircleImage,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    cv::vector<cv::RotatedRect> minEllipse( contours.size() );
    int numCircles = 0;
    bool circleMask [contours.size()];
    for( int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 5 )
        {
          minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) ); 
          minEllipse[i].center.x = minEllipse[i].center.x * processingScale;
          minEllipse[i].center.y = minEllipse[i].center.y * processingScale;
          minEllipse[i].size.height = minEllipse[i].size.height * processingScale;
          minEllipse[i].size.width = minEllipse[i].size.width * processingScale;
          double approxCircle = minEllipse[i].size.height/minEllipse[i].size.width;
          if (approxCircle <majorToMinor && approxCircle > 1/majorToMinor)
          {
            numCircles++;
            circleMask[i] = true;
            cv::Scalar color = cv::Scalar( 255,255,0 );
            cv::ellipse( cv_ptr->image, minEllipse[i], color, 2, 8 );
          }
        }
    }


//    std::cout << "Sizes: " << minEllipse.size() << ":"<< numCircles << std::endl;

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> r;

    if (numCircles>4 && numCircles<20 && minEllipse.size()<35) // culls out larger data sets
    {
      bool flag = false;
      for (int i = 0; i<minEllipse.size();i++)
      {
        if (circleMask[i])
        {
            for (int j=0; j<minEllipse.size();j++)
            {
              if (circleMask[j] && i!=j && ImageConverter::Distance(minEllipse[i].center.x,minEllipse[i].center.y,minEllipse[j].center.x,minEllipse[j].center.y) < 5)
              {
                    // std::cout << i << ":" << j << "::::"<< abs( (minEllipse[i].size.height>minEllipse[j].size.height ? minEllipse[i].size.height/minEllipse[j].size.height : minEllipse[j].size.height/minEllipse[i].size.height)  - 2) << std::endl;
                double r0 = minEllipse[i].size.height>minEllipse[i].size.width? minEllipse[i].size.height : minEllipse[i].size.width;
                double r1 = minEllipse[j].size.height>minEllipse[j].size.width? minEllipse[j].size.height : minEllipse[j].size.width;
                double diff = (r0>r1?r0/r1:r1/r0)-2;
                if ((diff>0?diff:-diff) <0.25)
                {
                  //std::cout << r0 << ":" << r1 << "::::"<< (diff>0?diff:-diff) << std::endl;
                  // calculates distance between points
                  cv::Scalar color = cv::Scalar( 255,255,255 );
                  cv::ellipse( cv_ptr->image, minEllipse[i], color, 2, 8 );
                  cv::ellipse( cv_ptr->image, minEllipse[j], color, 2, 8 );
                  bool success = true;
                  for (int k=0;k<x.size();k++)
                  {
                    if( abs(x.at(k)-minEllipse[i].center.x)<10 && abs(y.at(k)-minEllipse[i].center.y)<10 )
                    {success=false;}
                  }
                  if(success)
                  {
                    x.push_back(minEllipse[i].center.x);
                    y.push_back(minEllipse[i].center.y);
                    r.push_back(r0>r1?r0:r1);
                  }
                  if (x.size() == 4)
                  {
                    std::cout << "Points: " << x.size() << std::endl;
                    for (int index = 0; index < x.size(); index++)
                    {
                      circle_data.data[3*index]=x.at(index);
                      circle_data.data[3*index+1]=y.at(index);
                      circle_data.data[3*index+2]=r.at(index);
                    }
                    circle_pub.publish(circle_data);
                    flag=true;
                    break;
                  }
                }
              }
            }

          }
          if (flag==true){break;}
        }
      }
    

    cv::imshow(WINDOW, cv_ptr->image);
     //std::cout<<"lbl 2"<<std::endl;
    cv::waitKey(1);
    // std::cout<<"lbl 3"<<std::endl;
    
    image_pub_.publish(cv_ptr->toImageMsg());
     //std::cout<<"lbl 4"<<std::endl;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processor");
  ImageConverter ic;
  ros::spin();
  return 0;
} 

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
   cv::vector<cv::Vec3f> circles;
   cv::imshow(WINDOW, CircleImage);
   cv::HoughCircles( CircleImage, circles, CV_HOUGH_GRADIENT, 1, CircleImage.rows/(8*processingScale), 200, 100, 0, 0 );

    // cv::vector<std::vector<cv::Point> > contours;
    // cv::vector<cv::Vec4i> hierarchy;
    // cv::Canny(CircleImage,CircleImage,100,300,3);
    // //cv::imshow(WINDOW, CircleImage);
    // cv::findContours(CircleImage,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    // cv::vector<cv::RotatedRect> minEllipse( contours.size() );
    // int numCircles = 0;
    // bool circleMask [contours.size()];
    // for( int i = 0; i < contours.size(); i++ )
    // {
    //    if( contours[i].size() > 5 )
    //    {
    //       minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) ); 
    //       minEllipse[i].center.x = minEllipse[i].center.x * processingScale;
    //       minEllipse[i].center.y = minEllipse[i].center.y * processingScale;
    //       minEllipse[i].size.height = minEllipse[i].size.height * processingScale;
    //       minEllipse[i].size.width = minEllipse[i].size.width * processingScale;
    //       double approxCircle = minEllipse[i].size.height/minEllipse[i].size.width;
    //       if (approxCircle <1.2 && approxCircle > 0.8)
    //       {
    //         numCircles++;
    //         circleMask[i] = true;
    //         cv::Scalar color = cv::Scalar( 255,255,0 );
    //         cv::ellipse( cv_ptr->image, minEllipse[i], color, 2, 8 );
    //       }
    //     }
    // }


    // std::cout << "Sizes: " << minEllipse.size() << ":"<< numCircles << std::endl;

    // if (numCircles>4 && numCircles<10 && minEllipse.size()<15)
    // {
    //   int p1 = 0;
    //   int p2 = 0;
    //   int p3 = 0;
    //   int p4 = 0;
    //   for (int i = 0; i<minEllipse.size();i++)
    //   {
    //     if (circleMask[i])
    //     {
    //         std::vector<double> distances; // only tracks for the current i--in theory it will reach a solution rather quickly and not itterate through all but three
    //         for (int j=0; j<minEllipse.size();j++)
    //         {
    //           if (circleMask[j])
    //           {
    //             if (abs(minEllipse[i].size.height-minEllipse[j].size.height) < 100)
    //             {
    //                 distances.push_back(ImageConverter::Distance(minEllipse[i].center.x,minEllipse[i].center.y,minEllipse[j].center.x,minEllipse[j].center.y)); // calculates distance between points
                    
    //             }
    //             else
    //             {
    //               distances.push_back(0.0);
    //             }
    //             std::cout<<i<<":"<<j<<":"<<distances.at(j)<<std::endl;
    //           }
    //         }
    //         std::cout<<"i:::"<<numCircles<<":"<<distances.size()<<std::endl;
    //         for (int j=0;j<distances.size();j++)
    //         {
    //           if (i!=j && distances.at(j)>0.1)
    //           {
    //             for (int k=0;k<distances.size();k++)
    //             {
    //               if (j!=k&&i!=k && distances.at(k)>0.1)
    //               {
    //                 if (abs(distances.at(k)-distances.at(j))<0.5) // checks for repeated side distance
    //                 {
    //                   for (int l=0;l<distances.size();l++)
    //                   {
    //                     if (l!=i&&l!=j&&l!=k && distances.at(l)>0.1)
    //                     {
    //                       if ( abs( distances.at(l)-(distances.at(j)*sqrt(2)) )<20 )
    //                       {
    //                           std::cout<<i<<":"<<j<<":"<<k<<":"<<l<<std::endl;
    //                           break;
    //                       }
    //                     }
    //                   }
    //                 }
    //               }
    //             }
    //           }
    //         }

    //       }
    //     }
    //   }
     //std::cout<<"lbl 1"<<std::endl;
    // else if (numCircles==4)
    // {
    //   int j = 0;
    //   for (int i = 0; i<contours.size();i++)
    //   {
    //     if (circleMask[i])
    //     {
    //            circle_data.data[3*j]=minEllipse[i].center.x;
    //            circle_data.data[3*j+1]=minEllipse[i].center.y;
    //            circle_data.data[3*j+2]=minEllipse[i].size.height;
    //            j++;
    //     }
    //   }
    //   circle_pub.publish(circle_data);
    // }



    // Draw the circles detected
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

    std::cout << "circles" << circles.size() << std::endl;
    

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

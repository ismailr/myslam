#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <climits>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>

#include <opencv2/imgproc/imgproc.hpp>

void depth_cb (const sensor_msgs::Image::Ptr&);
void image_cb (const sensor_msgs::Image::Ptr&);

ros::Publisher pub_image_depth;
ros::Publisher pub_image;


int main (int argc, char** argv)
{
	ros::init(argc,argv,"front_end");
	ros::NodeHandle nh;

	ros::Subscriber sub_depth = nh.subscribe ("depth", 1, depth_cb);
	ros::Subscriber sub_image = nh.subscribe ("image", 1, image_cb);
	pub_image_depth = nh.advertise<sensor_msgs::Image> ("depth_out",1);
	pub_image = nh.advertise<sensor_msgs::Image> ("rgb_out",1);

	ros::spin();

	return 0;
}

cv_bridge::CvImagePtr img_ptr;

void image_cb (const sensor_msgs::Image::Ptr& image)
{
    try
    {
        img_ptr = cv_bridge::toCvCopy (image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {

    }
}

std::vector<cv_bridge::CvImagePtr> n_depth_frames(6);
static index = 0;

void depth_cb (const sensor_msgs::Image::Ptr& image)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy (image, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::medianBlur (cv_ptr->image, cv_ptr->image, 3);

        // buffering 6 frames
        n_depth_frames [ index % 6 ] = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {

    }

    int row = cv_ptr->image.rows;
    int col = cv_ptr->image.cols;

    ushort* d;
    ushort* _d;
    ushort* d_;

    uchar* m;
    uchar* _m;
    uchar* m_;

    ushort* x;

    for (int i = 1; i < row - 1; i++)
    {
        _d = cv_ptr->image.ptr<ushort>(i - 1);
        d = cv_ptr->image.ptr<ushort>(i);
        d_ = cv_ptr->image.ptr<ushort>(i + 1);

        _m = img_ptr->image.ptr<uchar>(i - 1);
        m = img_ptr->image.ptr<uchar>(i);
        m_ = img_ptr->image.ptr<uchar>(i + 1);

        for (int j = 1; j < col - 1 ; j++)
        {
            

            // normal estimation
            double dzdx = (d[j + 1] - d[j - 1])/2;
            double dzdy = (d_[j] - _d[j])/2;

            double mag = sqrt (dzdx*dzdx + dzdy*dzdy + 1);
            Eigen::Vector3f normal (-dzdx/mag, -dzdy/mag, 1/mag);

            m[j * 3] = normal[0] * 255;
            m[j * 3 + 1] = normal[1] * 255;
            m[j * 3 + 2] = normal[2] * 255;
        }
    }

    index++;

	pub_image.publish(img_ptr->toImageMsg());
	pub_image_depth.publish(cv_ptr->toImageMsg());
}

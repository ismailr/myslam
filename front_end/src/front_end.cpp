#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <climits>
#include <ctime>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/core.hpp>

//using namespace cv::saliency;

void depth_cb (const sensor_msgs::Image::Ptr&);
void image_cb (const sensor_msgs::Image::Ptr&);
void odom_cb (const nav_msgs::Odometry::Ptr&);

ros::Publisher pub_depth;
ros::Publisher pub_image;

cv_bridge::CvImagePtr img_ptr;

int nbuff = 1;
std::vector<cv_bridge::CvImagePtr> buffer(nbuff);

static int indeks = 0;
static int keyframe = 0;

int main (int argc, char** argv)
{
	ros::init(argc,argv,"front_end");
	ros::NodeHandle nh;

	ros::Subscriber sub_depth = nh.subscribe ("depth", 1, depth_cb);
	ros::Subscriber sub_image = nh.subscribe ("image", 1, image_cb);
    ros::Subscriber sub_odom = nh.subscribe ("odom", 1, odom_cb);
	pub_depth = nh.advertise<sensor_msgs::Image> ("depth_out",1);
	pub_image = nh.advertise<sensor_msgs::Image> ("rgb_out",1);

	ros::spin();

	return 0;
}

//cv_bridge::CvImagePtr bing_ptr;
//cv::String training_path = "/home/ism/data/datset/cv/pascal/voc_2007/VOC2007";
//cv::String training_path = "/home/ism/work/kinetic/src/opencv3/opencv_contrib/saliency/samples/ObjectnessTrainedModel";

cv::Mat gray, edge, draw;
cv_bridge::CvImagePtr edge_ptr;
void image_cb (const sensor_msgs::Image::Ptr& image)
{
//    cv::Ptr<Saliency> saliencyAlgorithm = Saliency::create("BING");

    try
    {
        img_ptr = cv_bridge::toCvCopy (image, sensor_msgs::image_encodings::BGR8);
        edge_ptr = cv_bridge::toCvCopy (image, sensor_msgs::image_encodings::MONO8);
//        cv::cvtColor (edge_ptr->image, gray, CV_BGR2GRAY);
        cv::Canny (edge_ptr->image, edge_ptr->image, 50, 150);
//        edge_ptr->image = draw;
//        bing_ptr = cv_bridge::toCvCopy (image, sensor_msgs::image_encodings::BGR8);

//        std::vector<cv::Vec4i> saliencyMap;
//        saliencyAlgorithm.dynamicCast<ObjectnessBING>()->setTrainingPath ( training_path );
//        saliencyAlgorithm.dynamicCast<ObjectnessBING>()->setBBResDir ( training_path + "/Results" );

//        std::cout << saliencyMap.size() << std::endl;

//        if ( keyframe % 2 == 0 && saliencyAlgorithm->computeSaliency (bing_ptr->image, saliencyMap))
//        {
//
//        }

    }
    catch (cv_bridge::Exception& e)
    {

    }
}


void depth_cb (const sensor_msgs::Image::Ptr& depth)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy (depth, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::medianBlur (cv_ptr->image, cv_ptr->image, 3);
//        cv::GaussianBlur (cv_ptr->image, cv_ptr->image, cv::Size(5,5), 1);

        buffer [indeks % nbuff] = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {

    }

    int row = cv_ptr->image.rows;
    int col = cv_ptr->image.cols;

    uchar *m;
    uchar *ed;

    Eigen::Vector3d x (1,0,0);
    Eigen::Vector3d y (0,1,0);
    Eigen::Vector3d z (0,0,1);


    int start = clock();
    if (indeks > nbuff - 1)
    {
        for (int i = 1; i < row - 1; i++)
        {
            m = img_ptr->image.ptr<uchar>(i);
            ed = edge_ptr->image.ptr<uchar>(i);

            for (int j = 1; j < col - 1; j++)
            {
                //      | n | 
                //    w | x | e
                //      | s | 
                double n = 0.0, e = 0.0, s = 0.0, w = 0.0;

                for (int k = 0; k < nbuff; ++k)
                {
                    n += (buffer[k]->image.ptr<ushort>(i - 1))[j];
                    e += (buffer[k]->image.ptr<ushort>(i))[j + 1];
                    s += (buffer[k]->image.ptr<ushort>(i + 1))[j];
                    w += (buffer[k]->image.ptr<ushort>(i))[j - 1];
                }

                double dzdx = (e - w)/(2 * nbuff);
                double dzdy = (s - n)/(2 * nbuff);

                double mag = sqrt (dzdx*dzdx + dzdy*dzdy + 1.0);
                Eigen::Vector3d normal (-dzdx/mag, -dzdy/mag, 1.0/mag); 
                
//                /* ********** normal estimation ********** */ 
//                _d  = cv_ptr->image.ptr<ushort>(i - 1);
//                d   = cv_ptr->image.ptr<ushort>(i);
//                d_  = cv_ptr->image.ptr<ushort>(i + 1);
//
//                double dzdx = (d[j + 1] - d[j - 1])/2;
//                double dzdy = (d_[j] - _d[j])/2;
//
//                double mag = sqrt (dzdx*dzdx + dzdy*dzdy + 1);
//                Eigen::Vector3d normal (-dzdx/mag, -dzdy/mag, 1/mag); 
//                /* ********** end of normal estimation ********** */

                m[j * 3] = normal[0] * 255;
                m[j * 3 + 1] = normal[1] * 255;
                m[j * 3 + 2] = normal[2] * 0;

                if (ed[j] == 255)
                {
                    m[j*3] = 255;
                    m[j*3 + 1] = 255;
                    m[j*3 + 2] = 255;
                }
            }
        }

//        cv::GaussianBlur (img_ptr->image, img_ptr->image, cv::Size(5,5), 1);

        pub_image.publish(img_ptr->toImageMsg());
//        pub_image.publish(edge_ptr->toImageMsg());
        pub_depth.publish(cv_ptr->toImageMsg());
    }

    int stop = clock();
    std::cout << (stop - start)/double(CLOCKS_PER_SEC) * 1000 << std::endl;

    ++indeks ;
    ++keyframe;
}

void odom_cb (const nav_msgs::Odometry::Ptr& odom)
{
    std::cout << "HORE" << std::endl;

}

#ifndef _LOGCAM_PLUGIN_H_
#define _LOGCAM_PLUGIN_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "myslam_sim_gazebo/LogicalImage.h"
#include "myslam_sim_gazebo/settings.h"

#include "mrpt_msgs/ObservationRangeBearing.h"

namespace gazebo {
    class LogicalCameraPlugin : public SensorPlugin {
        public:
            LogicalCameraPlugin() : SensorPlugin() {};
            virtual ~LogicalCameraPlugin() {};

            virtual void Load (sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        protected:
            ros::NodeHandle *nh;
            ros::Publisher image_pub;
	    ros::Publisher landmark_pub;

        private:
            virtual void onUpdate();

            sensors::LogicalCameraSensorPtr parentSensor;
            event::ConnectionPtr updateConnection;
    };
}

#endif

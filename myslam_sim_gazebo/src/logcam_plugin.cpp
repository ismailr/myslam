#include "myslam_sim_gazebo/logcam_plugin.h"
#include "myslam_system/helpers.h"

#include "tf/tf.h"

#include <string>

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN (LogicalCameraPlugin);

void LogicalCameraPlugin::Load (sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    this->parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

    if (!this->parentSensor) {
        gzerr << "LogicalCameraPlugin requires a Logical Camera Sensor.\n";
        return;
    }

    this->updateConnection = this->parentSensor->ConnectUpdated (
            std::bind (&LogicalCameraPlugin::onUpdate, this));

    this->parentSensor->SetActive (true);

    nh = new ros::NodeHandle();
    image_pub = nh->advertise<myslam_sim_gazebo::LogicalImage>("logical_camera_image",1,true);
    landmark_pub = nh->advertise<mrpt_msgs::ObservationRangeBearing>("landmark",1,true);
}

void LogicalCameraPlugin::onUpdate ()
{

	ros::Time now = ros::Time::now();

	msgs::LogicalCameraImage logical_image;
	myslam_sim_gazebo::LogicalImage msg;

	logical_image = this->parentSensor->Image();
	gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();

	if (!scene || !scene->Initialized())
	return;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "logical_camera_link";

	msg.pose.position.x = logical_image.pose().position().x();
	msg.pose.position.y = logical_image.pose().position().y();
	msg.pose.position.z = logical_image.pose().position().z();

	msg.pose.orientation.x = logical_image.pose().orientation().x();
	msg.pose.orientation.y = logical_image.pose().orientation().y();
	msg.pose.orientation.z = logical_image.pose().orientation().z();
	msg.pose.orientation.w = logical_image.pose().orientation().w();

    /* PUBLISHED FOR MRPT SLAM */
	mrpt_msgs::ObservationRangeBearing landmarks;
	landmarks.header.stamp = now;
	landmarks.header.frame_id = "base_link";

	landmarks.sensor_pose_on_robot.position.x = 0.0;
	landmarks.sensor_pose_on_robot.position.y = 0.0;
	landmarks.sensor_pose_on_robot.position.z = 0.0;
	landmarks.sensor_pose_on_robot.orientation.x = 0.0;
	landmarks.sensor_pose_on_robot.orientation.y = 0.0;
	landmarks.sensor_pose_on_robot.orientation.z = 0.0;
	landmarks.sensor_pose_on_robot.orientation.w = 0.0;

	landmarks.min_sensor_distance = 0.5;
	landmarks.max_sensor_distance = 5.0;

	landmarks.sensor_std_range = 0.0;
	landmarks.sensor_std_yaw = 0.0;
	landmarks.sensor_std_pitch = 0.0;
	/* END FOR MRPT */

	int n = logical_image.model_size();

	for (int i = 0; i < n; i++) {
		myslam_sim_gazebo::Model model_msg;

		if (logical_image.model(i).name() == "ground_plane" || logical_image.model(i).name() == "indoor1")
		    continue;

		rendering::VisualPtr visual = scene->GetVisual (logical_image.model(i).name());

		if (!visual)
		    continue;

		//        math::Box bounding_box = visual->GetBoundingBox();

		// add some noise
		double x = logical_image.model(i).pose().position().x();
		double y = logical_image.model(i).pose().position().y();
		double z = logical_image.model(i).pose().position().z();

		double range = sqrt (x*x + y*y);
		double bearing = atan2 (y,x);

		double std_r = MYSLAM::B1 * range; 
		double std_b = MYSLAM::B2 * bearing;

		double range_noise = gaussian_generator<double>(0.0, std_r);
		double bearing_noise = gaussian_generator<double>(0.0, std_b);

		range += range_noise;
		bearing += bearing_noise;

		x += range * cos (bearing);
		y += range * sin (bearing);

		model_msg.pose.position.x = x; 
		model_msg.pose.position.y = y;
		model_msg.pose.position.z = z;

		tf::Quaternion q (
				logical_image.model(i).pose().orientation().x(),	
				logical_image.model(i).pose().orientation().y(),	
				logical_image.model(i).pose().orientation().z(),	
				logical_image.model(i).pose().orientation().w());
		double roll, pitch, yaw;
		tf::Matrix3x3 (q).getRPY (roll,pitch,yaw);

		double std_orientation = MYSLAM::B3 * range + MYSLAM::B4 * bearing + MYSLAM::B5 * yaw;

		double orientation_noise = gaussian_generator<double>(0.0, std_orientation);

		yaw += orientation_noise;
		q.setRPY (roll,pitch,yaw);

		model_msg.pose.orientation.x = q.x();
		model_msg.pose.orientation.y = q.y();
		model_msg.pose.orientation.z = q.z();
		model_msg.pose.orientation.w = q.w();

		//        model_msg.pose.orientation.x = logical_image.model(i).pose().orientation().x();
		//        model_msg.pose.orientation.y = logical_image.model(i).pose().orientation().y();
		//        model_msg.pose.orientation.z = logical_image.model(i).pose().orientation().z();
		//        model_msg.pose.orientation.w = logical_image.model(i).pose().orientation().w();

		//        model_msg.size.x = bounding_box.GetXLength();
		//        model_msg.size.y = bounding_box.GetYLength();
		//        model_msg.size.z = bounding_box.GetZLength();
		//
		//        model_msg.min.x = bounding_box.GetCenter().x - bounding_box.GetSize().x/2.0;
		//        model_msg.min.y = bounding_box.GetCenter().y - bounding_box.GetSize().y/2.0;
		//        model_msg.min.z = bounding_box.GetCenter().z - bounding_box.GetSize().z/2.0;
		//
		//        model_msg.max.x = bounding_box.GetCenter().x + bounding_box.GetSize().x/2.0;
		//        model_msg.max.y = bounding_box.GetCenter().y + bounding_box.GetSize().y/2.0;
		//        model_msg.max.z = bounding_box.GetCenter().z + bounding_box.GetSize().z/2.0;

		model_msg.name = logical_image.model(i).name();
		model_msg.type = logical_image.model(i).name().substr(0,3);
		msg.models.push_back (model_msg);

		mrpt_msgs::SingleRangeBearingObservation landmark;
		landmark.range = sqrt (x*x + y*y);
		landmark.yaw = atan2(y,x);
		landmark.pitch = 0.0;
		landmark.id = -1;
		landmarks.sensed_data.push_back (landmark);
	}

	this->image_pub.publish (msg);
	this->landmark_pub.publish (landmarks);
}

#include "myslam_sim_gazebo/logcam_plugin.h"

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
}

void LogicalCameraPlugin::onUpdate ()
{
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

    int n = logical_image.model_size();

    for (int i = 0; i < n; i++) {
        myslam_sim_gazebo::Model model_msg;

        if (logical_image.model(i).name() == "ground_plane" || logical_image.model(i).name() == "indoor1")
            continue;

        rendering::VisualPtr visual = scene->GetVisual (logical_image.model(i).name());

        if (!visual)
            continue;

        math::Box bounding_box = visual->GetBoundingBox();

        model_msg.pose.position.x = logical_image.model(i).pose().position().x();
        model_msg.pose.position.y = logical_image.model(i).pose().position().y();
        model_msg.pose.position.z = logical_image.model(i).pose().position().z();

        model_msg.pose.orientation.x = logical_image.model(i).pose().orientation().x();
        model_msg.pose.orientation.y = logical_image.model(i).pose().orientation().y();
        model_msg.pose.orientation.z = logical_image.model(i).pose().orientation().z();
        model_msg.pose.orientation.w = logical_image.model(i).pose().orientation().w();

        model_msg.size.x = bounding_box.GetXLength();
        model_msg.size.y = bounding_box.GetYLength();
        model_msg.size.z = bounding_box.GetZLength();

        model_msg.min.x = bounding_box.GetCenter().x - bounding_box.GetSize().x/2.0;
        model_msg.min.y = bounding_box.GetCenter().y - bounding_box.GetSize().y/2.0;
        model_msg.min.z = bounding_box.GetCenter().z - bounding_box.GetSize().z/2.0;

        model_msg.max.x = bounding_box.GetCenter().x + bounding_box.GetSize().x/2.0;
        model_msg.max.y = bounding_box.GetCenter().y + bounding_box.GetSize().y/2.0;
        model_msg.max.z = bounding_box.GetCenter().z + bounding_box.GetSize().z/2.0;

        model_msg.name = logical_image.model(i).name();
        model_msg.type = logical_image.model(i).name().substr(0,3);
        msg.models.push_back (model_msg);
    }

    this->image_pub.publish (msg);
}

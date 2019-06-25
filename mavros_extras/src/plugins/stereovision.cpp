#include <mavros/mavros_plugin.h>
#include <iostream>
#include <mavros_msgs/StereoVision.h>

namespace mavros {
namespace extra_plugins {
class StereoVisionPlugin : public plugin::PluginBase {
public:
    StereoVisionPlugin() : PluginBase(),
        nh("~stereo_vision")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        stereo_vision_sub = nh.subscribe("odometry", 10, &StereoVisionPlugin::stereo_vision_cb, this);
    };

    Subscriptions get_subscriptions()
    {
        return {/* no subscriptions */};
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber stereo_vision_sub;

    void stereo_vision_cb(const mavros_msgs::StereoVision::ConstPtr &msg)
    {
        mavlink::ardupilotmega::msg::STEREO_VISION_ODOM svo {};

        svo.time_usec = msg->time_usec;
        svo.time_delta_usec = msg->time_delta_usec;

        svo.distance = msg->distance;

        svo.delta_pitch = msg->delta_pitch;
        svo.delta_yaw = msg->delta_yaw;

        svo.lin_velocity[0] = float(msg->v_body.x);
        svo.lin_velocity[1] = float(msg->v_body.y);
        svo.lin_velocity[2] = float(msg->v_body.z);

        svo.mesh_count = msg->mesh_count;

        UAS_FCU(m_uas)->send_message_ignore_drop(svo);
    }
};
};
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::StereoVisionPlugin, mavros::plugin::PluginBase)

#include <mavros/mavros_plugin.h>
#include <iostream>
#include <mavros_msgs/StereoVision.h>
#include <mavros_msgs/NetInspection.h>
#include <mavros_msgs/PhaseCorr.h>
#include <mavros_msgs/NetTrackingMarker.h>

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
        stereo_vision_sub = nh.subscribe("net_tracking", 10, &StereoVisionPlugin::stereo_vision_cb, this);
        net_insp_sub = nh.subscribe("net_inspection", 10, &StereoVisionPlugin::net_insp_cb, this);
        phase_corr_sub = nh.subscribe("phase_corr", 10, &StereoVisionPlugin::phase_corr_cb, this);
        nettr_marker_sub = nh.subscribe("optical_marker", 10, &StereoVisionPlugin::optical_marker_cb, this);
    };

    Subscriptions get_subscriptions()
    {
        return {/* no subscriptions */};
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber stereo_vision_sub;
    ros::Subscriber net_insp_sub;
    ros::Subscriber phase_corr_sub;
    ros::Subscriber nettr_marker_sub;

    void stereo_vision_cb(const mavros_msgs::StereoVision::ConstPtr &msg)
    {
        mavlink::ardupilotmega::msg::STEREO_VISION_ODOM svo {};

        svo.time_usec = msg->time_usec;
        svo.time_delta_usec = msg->time_delta_usec;

        // distance to tracked surface
        svo.distance = msg->distance;

        // angle offset to tracked surface (radians)
        svo.delta_pitch = msg->delta_pitch;
        svo.delta_yaw = msg->delta_yaw;

        UAS_FCU(m_uas)->send_message_ignore_drop(svo);
    }

    void net_insp_cb(const mavros_msgs::NetInspection::ConstPtr &msg)
    {
        mavlink::ardupilotmega::msg::NET_INSPECTION ni {};

        ni.time_usec = msg->time_usec;
        ni.time_delta_usec = msg->time_delta_usec;

        // amount and distribution of currently visible net meshes
        ni.mesh_count = msg->mesh_count;
        ni.mesh_distribution = msg->mesh_distribution;

        UAS_FCU(m_uas)->send_message_ignore_drop(ni);
    }

    void phase_corr_cb(const mavros_msgs::PhaseCorr::ConstPtr &msg)
    {
        mavlink::ardupilotmega::msg::PHASE_CORR pc {};

        pc.time_usec = msg->time_usec;
        pc.time_delta_usec = msg->time_delta_usec;

        // translational shift between current and previous image
        pc.phase_shift_x = msg->phase_shift_x;
        pc.phase_shift_y = msg->phase_shift_y;

        // accumulated translational shift wit regard to first received image
        pc.phase_shift_sum_x = msg->phase_shift_sum_x;
        pc.phase_shift_sum_y = msg->phase_shift_sum_y;

        UAS_FCU(m_uas)->send_message_ignore_drop(pc);

    }

    void optical_marker_cb(const mavros_msgs::NetTrackingMarker::ConstPtr &msg)
    {
        mavlink::ardupilotmega::msg::NETTRACKING_MARKER nm {};

        nm.time_usec = msg->time_usec;
        nm.time_delta_usec = msg->time_delta_usec;

        nm.marker_visible = msg->marker_visible ? 1 : 0;
        nm.terminate = msg->terminate ? 1 : 0;
        nm.horizontal_pos = msg->horizontal_pos;

        UAS_FCU(m_uas)->send_message_ignore_drop(nm);

    }
};
};
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::StereoVisionPlugin, mavros::plugin::PluginBase)

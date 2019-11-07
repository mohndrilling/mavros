#include <mavros/mavros_plugin.h>
#include <iostream>
#include <mavros_msgs/NetTrackingState.h>

namespace mavros {
namespace extra_plugins {
class NetTrackingPlugin : public plugin::PluginBase {
public:
    NetTrackingPlugin() : PluginBase(),
        nh("~net_tracking")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        net_tracking_pub = nh.advertise<mavros_msgs::NetTrackingState>("state", 10);
    };

    Subscriptions get_subscriptions()
    {
        return {
            make_handler(&NetTrackingPlugin::handle_nettr_state)
        };
    }

private:
    ros::NodeHandle nh;
    ros::Publisher net_tracking_pub;

    /**
     * @brief Handle NETTRACKING_STATE MAVlink message.
     * @param msg	Received Mavlink msg
     * @param nettr_state	NETTRACKING_STATE msg
     */
    void handle_nettr_state(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::NETTRACKING_STATE &nettr_state)
    {
        //publish net tracking state
        mavros_msgs::NetTrackingState nettr_state_msg;
        nettr_state_msg.state = nettr_state.state;
        nettr_state_msg.loop_progress = nettr_state.loop_progress;

        net_tracking_pub.publish(nettr_state_msg);
    }
};
};
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::NetTrackingPlugin, mavros::plugin::PluginBase)

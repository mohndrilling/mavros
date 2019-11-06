#include <mavros/mavros_plugin.h>
#include <iostream>
#include <std_msgs/UInt8.h>

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
        net_tracking_pub = nh.advertise<std_msgs::UInt8>("state", 10);
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
        std_msgs::UInt8 nettr_state_msg;
        nettr_state_msg.data = nettr_state.state;

        net_tracking_pub.publish(nettr_state_msg);
    }
};
};
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::NetTrackingPlugin, mavros::plugin::PluginBase)

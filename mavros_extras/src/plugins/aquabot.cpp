#include <mavros/mavros_plugin.h>
#include <iostream>
#include <mavros_msgs/NetTrackingState.h>
#include "std_srvs/SetBool.h"

namespace mavros {
namespace extra_plugins {
class AquaBotPlugin : public plugin::PluginBase {
public:
    AquaBotPlugin() : PluginBase(),
        nh()
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        net_tracking_pub = nh.advertise<mavros_msgs::NetTrackingState>("/mavros/net_tracking/state", 10);
        set_brush_motors_client = nh.serviceClient<std_srvs::SetBool>("/aquabot/set_brush_motors");

        brush_motors_running = false;
    };

    Subscriptions get_subscriptions()
    {
        return {
            make_handler(&AquaBotPlugin::handle_nettr_state),
            make_handler(&AquaBotPlugin::handle_netcl_state)
        };
    }

private:
    ros::NodeHandle nh;
    ros::Publisher net_tracking_pub;
    ros::ServiceClient set_brush_motors_client;

    bool brush_motors_running;

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

    /**
     * @brief Handle NETCLEANING_STATE MAVlink message.
     * @param msg	Received Mavlink msg
     * @param netcl_state	NETCLEANING_STATE msg
     */
    void handle_netcl_state(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::NETCLEANING_STATE &netcl_state)
    {
        // whether the brush motors need to run during the current net cleaning state
        bool brush_motors_required = netcl_state.brush_motors_active == 1;

        if (brush_motors_required != brush_motors_running)
        {
            std_srvs::SetBool srv;
            srv.request.data = brush_motors_required;

            if (set_brush_motors_client.call(srv))
                brush_motors_running = brush_motors_required;
        }
    }
};
};
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::AquaBotPlugin, mavros::plugin::PluginBase)

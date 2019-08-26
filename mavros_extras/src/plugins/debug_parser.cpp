#include <mavros/mavros_plugin.h>
#include "std_msgs/Float32.h"
#include <mavros_msgs/DebugValue.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

/* MavLink contains some common debugging messages. In the flight controller stack, you can send arbitrary values via mavlink, e.g. following the function:
 * GCS::send_named_float(const char *name, float value).
 * In any arbitrary cpp file, be sure to have the include statement: #include <GCS_MAVLink/GCS.h>
 * Then e.g. send a float variable following the line:
 * gcs().send_named_float("some_unique_name", debug_value).
 *
 * The package mavros_extras contains the DebugValuePlugin by default, which listens to various common mavlink debug messages and publishes them on custom topics.
 * E.g. the mavlink messages 'named_value_float' will be published on the topic /mavros/debug_value/named_value_float.
 *
 * This node listens to this topic and republishes the received value, but assigning a distinct topic name to each debug variable.
 * Following the above gcs().send_named_float(..) line, it is now possible, e.g. to plot this distinct variable by running rqt_plot and entering the topic /mavros/debug_value/parsed/some_unique_name.
 */

namespace mavros {
namespace extra_plugins {
class DebugParserPlugin : public plugin::PluginBase {
public:
    DebugParserPlugin() : PluginBase(),
        nh("~debug_value")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        debug_sub = nh.subscribe("named_value_float", 1000, &DebugParserPlugin::debug_cb, this);

    };

    Subscriptions get_subscriptions()
    {
        return {/* no subscriptions */};
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber debug_sub;
    std::map<std::string, ros::Publisher> debug_publisher; /// dynamically sized map, mapping each debugValue-name to a corresponding publisher
    std::list<std::string> debug_fields;                   /// list containing all received names of debugging variables

    void debug_cb(const mavros_msgs::DebugValue::ConstPtr& msg)
    {
        std::string name = msg->name.c_str();
        // check if a debug variable with the given name has been received before. If not, create a new publisher with a unique topic name corresponding to the variable name.
        bool entry_exists = (std::find(std::begin(debug_fields), std::end(debug_fields), name) != std::end(debug_fields));
        if (!entry_exists) {
            std::stringstream topic_name;
            topic_name << "parsed/" << name;
            debug_publisher[name] =nh.advertise<std_msgs::Float32>(topic_name.str(), 10);
            debug_fields.push_back(name);
        }

        std_msgs::Float32 f;
        f.data = msg->value_float;
        debug_publisher[name].publish(f);
    }
};
};
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::DebugParserPlugin, mavros::plugin::PluginBase)

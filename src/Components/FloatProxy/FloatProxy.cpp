/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#include <memory>
#include <string>

#include "FloatProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FloatProxy {

FloatProxy::FloatProxy(const std::string & name) :
		Base::Component(name) ,
		ros_topic_name("ros.topic_name", std::string("float")),
		ros_namespace("ros.namespace", std::string("discode")) {
		registerProperty(ros_topic_name);
		registerProperty(ros_namespace);

}

FloatProxy::~FloatProxy() {
}

void FloatProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);

	// Register handlers
	h_onNewData.setup(boost::bind(&FloatProxy::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
//	addDependency("onNewData", &in_data);
	addDependency("onNewData", NULL);

}

bool FloatProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<std_msgs::Float32>(ros_topic_name, 1000);
	sub = nh->subscribe("my_topic", 1, &FloatProxy::callback, this);
	return true;
}

bool FloatProxy::onFinish() {
	delete nh;
	return true;
}

bool FloatProxy::onStop() {
	return true;
}

bool FloatProxy::onStart() {
	return true;
}

void FloatProxy::onNewData() {
	std_msgs::Float32 msg;
//	msg.data = in_data.read();
	msg.data = 5.5;
	pub.publish(msg);
	ros::spinOnce();
}

void FloatProxy::callback(const std_msgs::Float32ConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data;
}


} //: namespace FloatProxy
} //: namespace Processors

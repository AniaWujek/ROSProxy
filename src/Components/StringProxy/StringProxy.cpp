/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#include <memory>
#include <string>

#include "StringProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include "Common/Timer.hpp"

namespace Processors {
namespace StringProxy {

StringProxy::StringProxy(const std::string & name) :
		Base::Component(name) ,
		ros_topic_name("ros.topic_name", std::string("string")),
		ros_namespace("ros.namespace", std::string("namespace")) {
		registerProperty(ros_topic_name);
		registerProperty(ros_namespace);

}

StringProxy::~StringProxy() {
}

void StringProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);

	// Register handlers
	h_onNewData.setup(boost::bind(&StringProxy::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
	addDependency("onNewData", &in_data);
	//addDependency("onNewData", NULL);

}

bool StringProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<std_msgs::String>(ros_topic_name, 1000);
	//sub = nh->subscribe("my_topic", 1, &StringProxy::callback, this);
	return true;
}

bool StringProxy::onFinish() {
	delete nh;
	return true;
}

bool StringProxy::onStop() {
	return true;
}

bool StringProxy::onStart() {
	return true;
}

void StringProxy::onNewData() {

    Common::Timer t;
    t.restart();
    if(!in_data.empty()){

	    std::string data = in_data.read();
    
    	std_msgs::String msg;
    	msg.data = data;

		pub.publish(msg);
		ros::spinOnce();
		

		
    }
    CLOG(LNOTICE) << "Elapsed: " << t.elapsed();


}

void StringProxy::callback(const std_msgs::StringConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data.size();
}


} //: namespace StringProxy
} //: namespace Processors

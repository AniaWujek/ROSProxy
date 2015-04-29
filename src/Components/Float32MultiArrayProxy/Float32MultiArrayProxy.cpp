/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#include <memory>
#include <string>

#include "Float32MultiArrayProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include "Common/Timer.hpp"

namespace Processors {
namespace Float32MultiArrayProxy {

Float32MultiArrayProxy::Float32MultiArrayProxy(const std::string & name) :
		Base::Component(name) ,
		ros_topic_name("ros.topic_name", std::string("float32MultiArray")),
		ros_namespace("ros.namespace", std::string("discode_irp6_drawing")) {
		registerProperty(ros_topic_name);
		registerProperty(ros_namespace);

}

Float32MultiArrayProxy::~Float32MultiArrayProxy() {
}

void Float32MultiArrayProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);

	// Register handlers
	h_onNewData.setup(boost::bind(&Float32MultiArrayProxy::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
	addDependency("onNewData", &in_data);
	//addDependency("onNewData", NULL);

}

bool Float32MultiArrayProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<std_msgs::Float32MultiArray>(ros_topic_name, 1000);
	sub = nh->subscribe("my_topic", 1, &Float32MultiArrayProxy::callback, this);
	return true;
}

bool Float32MultiArrayProxy::onFinish() {
	delete nh;
	return true;
}

bool Float32MultiArrayProxy::onStop() {
	return true;
}

bool Float32MultiArrayProxy::onStart() {
	return true;
}
int i = 0;
void Float32MultiArrayProxy::onNewData() {

    Common::Timer t;
    t.restart();
    if(!in_data.empty()){

    std::vector<float> data = in_data.read();

    CLOG(LTRACE) << "\n\n CONTOURS SIZE: "<<data[0]<<"\n\n";

	std_msgs::Float32MultiArray msg;

	for(int i = 0; i < data.size(); ++i) {
        msg.data.push_back(data[i]);
	}

	pub.publish(msg);
	ros::spinOnce();
    }
    CLOG(LNOTICE) << "Elapsed: " << t.elapsed();


}

void Float32MultiArrayProxy::callback(const std_msgs::Float32MultiArrayConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data.size();
}


} //: namespace Float32MultiArrayProxy
} //: namespace Processors

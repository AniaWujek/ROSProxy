/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#include <memory>
#include <string>

#include "Float32MultiArrayProxy2.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include "Common/Timer.hpp"

namespace Processors {
namespace Float32MultiArrayProxy2 {

Float32MultiArrayProxy2::Float32MultiArrayProxy2(const std::string & name) :
		Base::Component(name) ,
		ros_topic_name("ros.topic_name", std::string("float32MultiArray")),
		ros_namespace("ros.namespace", std::string("discode_irp6_drawing")) {
		registerProperty(ros_topic_name);
		registerProperty(ros_namespace);

}

Float32MultiArrayProxy2::~Float32MultiArrayProxy2() {
}

void Float32MultiArrayProxy2::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);

	// Register handlers
	h_onNewData.setup(boost::bind(&Float32MultiArrayProxy2::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
	addDependency("onNewData", &in_data);

	//addDependency("onNewData", NULL);

}

bool Float32MultiArrayProxy2::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<std_msgs::Float32MultiArray>(ros_topic_name, 1000);
	//sub = nh->subscribe("my_topic", 1, &Float32MultiArrayProxy2::callback, this);
	return true;
}

bool Float32MultiArrayProxy2::onFinish() {
	delete nh;
	return true;
}

bool Float32MultiArrayProxy2::onStop() {
	return true;
}

bool Float32MultiArrayProxy2::onStart() {
	return true;
}

bool checkData(std::vector<std::vector<float> > &data) {
	if(data.size()>0) {
		int s = data[0].size();
		for(int i=1; i<data.size(); ++i) {
			if(data[i].size()!=s) return false;
		}
		return true;	
	}
	return false;
}

void Float32MultiArrayProxy2::onNewData() {

    Common::Timer t;
    t.restart();
    if(!in_data.empty()){

	    std::vector<std::vector<float> > data = in_data.read();
	    if(checkData(data)) {
	    	std_msgs::Float32MultiArray msg;
	    	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	    	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	    	msg.layout.dim[0].size = data.size();
	    	msg.layout.dim[0].stride = data.size()*data[0].size();
	    	msg.layout.dim[0].label = "rows";
	    	msg.layout.dim[1].size = data[0].size();
	    	msg.layout.dim[1].stride = data[0].size();
	    	msg.layout.dim[1].label = "cols";

	    	for(int i=0; i<data.size(); ++i) {
	    		for(int j=0; j<data[i].size(); ++j) {
	    			msg.data.push_back(data[i][j]);
	    		}
	    	}	    	

			/*for(int i = 0; i < data.size(); ++i) {
		        msg.data.push_back(data[i]);
			}*/

			pub.publish(msg);
			ros::spinOnce();
		}

		
    }
    CLOG(LNOTICE) << "Elapsed: " << t.elapsed();


}


void Float32MultiArrayProxy2::callback(const std_msgs::Float32MultiArrayConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data.size();
}


} //: namespace Float32MultiArrayProxy2
} //: namespace Processors

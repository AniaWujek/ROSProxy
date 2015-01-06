/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#ifndef Float32MultiArrayProxy_HPP_
#define Float32MultiArrayProxy_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

namespace Processors {
namespace Float32MultiArrayProxy {

/*!
 * \class Float32MultiArrayProxy
 * \brief Float32MultiArrayProxy processor class.
 *
 * Float32MultiArrayProxy processor.
 */
class Float32MultiArrayProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Float32MultiArrayProxy(const std::string & name = "Float32MultiArrayProxy");

	/*!
	 * Destructor
	 */
	virtual ~Float32MultiArrayProxy();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


// Input data streams

		Base::DataStreamIn< std::vector<float> > in_data;

// Output data streams

	// Handlers
	Base::EventHandler2 h_onNewData;
		Base::Property<std::string> ros_topic_name;
		Base::Property<std::string> ros_namespace;


	// Handlers
	void onNewData();

	ros::Publisher pub;
	ros::Subscriber sub;
	ros::NodeHandle * nh;

	void callback(const std_msgs::Float32MultiArrayConstPtr& msg);
};

} //: namespace Float32MultiArrayProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Float32MultiArrayProxy", Processors::Float32MultiArrayProxy::Float32MultiArrayProxy)

#endif /* Float32MultiArrayProxy_HPP_ */

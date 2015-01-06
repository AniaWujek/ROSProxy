/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#ifndef FloatProxy_HPP_
#define FloatProxy_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

namespace Processors {
namespace FloatProxy {

/*!
 * \class FloatProxy
 * \brief FloatProxy processor class.
 *
 * FloatProxy processor.
 */
class FloatProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FloatProxy(const std::string & name = "FloatProxy");

	/*!
	 * Destructor
	 */
	virtual ~FloatProxy();

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

		Base::DataStreamIn<float> in_data;

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

	void callback(const std_msgs::Float32ConstPtr& msg);
};

} //: namespace FloatProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FloatProxy", Processors::FloatProxy::FloatProxy)

#endif /* FloatProxy_HPP_ */

/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#ifndef StringProxy_HPP_
#define StringProxy_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace Processors {
namespace StringProxy {

/*!
 * \class StringProxy
 * \brief StringProxy processor class.
 *
 * StringProxy processor.
 */
class StringProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	StringProxy(const std::string & name = "StringProxy");

	/*!
	 * Destructor
	 */
	virtual ~StringProxy();

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

		Base::DataStreamIn< std::string > in_data;

// Output data streams

	// Handlers
	Base::EventHandler2 h_onNewData;
		Base::Property<std::string> ros_topic_name;
		Base::Property<std::string> ros_namespace;


	// Handlers
	void onNewData();

	ros::Publisher pub;
	//ros::Subscriber sub;
	ros::NodeHandle * nh;

	void callback(const std_msgs::StringConstPtr& msg);
};

} //: namespace StringProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("StringProxy", Processors::StringProxy::StringProxy)

#endif /* StringProxy_HPP_ */

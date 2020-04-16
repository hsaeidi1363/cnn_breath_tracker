/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/data_collector/qnode.hpp"
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace data_collector {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::getImage(const sensor_msgs::Image & _data){
	this->cam_img = _data;
	Q_EMIT imageUpdated(); 
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"data_collector");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	begin  = false;
	pause = false;
	stop = false;
        breathing_stopped = false;
        cmd_pub = n.advertise<data_collector::Cmd>("/data_collector/cmd", 10);
       
        image_sub = n.subscribe("/see_scope/nir/image_raw", 1, &QNode::getImage, this);


	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"data_collector");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	begin  = false;
	pause = false;
	stop = false;
        breathing_stopped = false;
        cmd_pub = n.advertise<data_collector::Cmd>("/data_collector/cmd", 10);

        image_sub = n.subscribe("/see_scope/nir/image_raw", 1, &QNode::getImage, this);	

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(20);
	int count = 0;
	data_collector::Cmd cmd_msg;
	while ( ros::ok() ) {

             if(breathing_stopped){
			cmd_msg.breathing_stopped = true;
			cmd_msg.start = true;
			cmd_pub.publish(cmd_msg);
			cmd_msg.breathing_stopped = false;
			cmd_msg.start = false;
			log(Info,std::string("Labeled one frame as when breathing stopped"));
			breathing_stopped = false;
  	     }

	     if(begin){
			cmd_msg.start = true;
			cmd_pub.publish(cmd_msg);
			cmd_msg.start = false;
			log(Info,std::string("Starting a set of data collection"));
			begin = false;
	     }

             if(pause){
			cmd_msg.pause = true;
			cmd_pub.publish(cmd_msg);
			cmd_msg.pause = false;
			log(Info,std::string("Pausing the data collection"));
			pause = false;
	     }

             if(stop){
			cmd_msg.stop = true;
			cmd_pub.publish(cmd_msg);
			cmd_msg.stop = false;
			log(Info,std::string("Ending a set of data collection"));
			stop = false;
	     }
		
             ros::spinOnce();
	     loop_rate.sleep();
             ++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace data_collector

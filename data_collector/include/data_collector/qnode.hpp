/**
 * @file /include/data_collector/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef data_collector_QNODE_HPP_
#define data_collector_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <data_collector/Cmd.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace data_collector {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
        void getImage(const sensor_msgs::Image & _data);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  
        sensor_msgs::Image cam_img;

	bool begin;
	bool breathing_stopped;
	bool stop;
	bool pause;


Q_SIGNALS:
	void loggingUpdated();
	void rosShutdown();
	void imageUpdated();

private:
	int init_argc;
	char** init_argv;

	ros::Publisher cmd_pub;

        ros::Subscriber image_sub;

    QStringListModel logging_model;
};

}  // namespace data_collector

#endif /* data_collector_QNODE_HPP_ */

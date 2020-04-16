#include<ros/ros.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>

#include<sensor_msgs/Image.h>
#include<data_collector/Cmd.h>

#include <string>
#include <fstream>

using namespace cv;
using namespace std;
using namespace cv_bridge;

//------- global variables

ros::Time t0;


// cv matrices for tracking the video frames
Mat img(cv::Size(128, 128), CV_8UC1); // image read from ROS topic and saved as a resized image

// cv pointer for reading ros image messages
CvImagePtr cv_ptr;
unsigned int width, height;

// initialization and state tracking variables
bool begin_labeling = false;
bool pause_labeling = false;
bool stop_labeling = false;
bool breathing_stopped = false;

bool image_received = false;


void get_img(const sensor_msgs::Image & _data){

  // extract the image and other info
  cv_ptr = toCvCopy(_data, "mono8");
  Mat src_img = cv_ptr->image;
 
  resize(src_img, img, img.size(), 0, 0, INTER_LINEAR);

//  img = img_tmp;

  // update the state variables
  if(!image_received){
	  std::cout << "Received the first input image with size " << _data.width << " by " << _data.height << std::endl;
	  image_received = true;
  }
}


int img_label = 0; // 0 means breathing, 1 means stopped breathing

void get_cmd(const data_collector::Cmd & _data){

	begin_labeling = _data.start;
	pause_labeling = _data.pause;
	stop_labeling = _data.stop;
	breathing_stopped = _data.breathing_stopped;

	if(begin_labeling && !breathing_stopped)
		std::cout << "Labeling started " << std::endl;

	if(pause_labeling)
		std::cout << "Labeling paused " << std::endl;

	if(stop_labeling)
		std::cout << "Labeling ended " << std::endl;

	if(breathing_stopped){
		std::cout << "Breathing stopped label received " << std::endl;
	}

}// end of get_cmd




int main(int argc, char * argv[]){
  //set the node
  ros::init(argc, argv, "labeler");
  ros::NodeHandle nh_;
  ros::NodeHandle home("~");

  string input_topic;
  string save_address;

  // get the ros params from the launch file
  home.getParam("input_topic", input_topic);
  home.getParam("save_address", save_address);


  
  int loop_freq = 7; //TODO: MAKE SURE THE NIR DATA IS ALWAYS COLLECTED AT 7 Hz
  ros::Rate loop_rate(loop_freq); 

  // define the ROS subscribers
  ros::Subscriber img_sub = nh_.subscribe(input_topic, 1,get_img);
  ros::Subscriber cmd_sub = nh_.subscribe("/data_collector/cmd", 1, get_cmd);


  std::ofstream labels_file;
  try{
      //labels_file.open("/home/hsaeidi/nn_data_for_breathing_tracker/set1/labels/lebels.csv");
	labels_file.open((save_address + "/labels/lebels.csv").c_str());
  }catch (...){
      std::cout << "Could not open the labels file (check the address)" <<std::endl;
  }
  int ctr = 1;
  int stopped_ctr = 0;
  while(ros::ok()){
    if(image_received && begin_labeling){
	    if(breathing_stopped){
			img_label = 1;
			stopped_ctr ++;
			if(stopped_ctr > 7){			
				breathing_stopped = false;
				stopped_ctr = 0;
			}
	    }else{
			img_label = 0;
	    }
	    labels_file << img_label << std::endl;
	    string img_name = (save_address + "/images/img" + std::to_string(ctr) + ".png" ).c_str();
	    std::cout << img_name <<std::endl;
	    imwrite( img_name, img );
	    ctr ++;
    }
    if(stop_labeling)
	break;
    ros::spinOnce();
    loop_rate.sleep();
  }
  labels_file.close();
  return 0;
}//end of main




#include<ros/ros.h>

#include<geometry_msgs/PolygonStamped.h>

#include <string>
#include <fstream>

using namespace std;

//------- global variables

ros::Time t0;


float x = 0.0;
float y = 0.0;

bool data_received = false;
 
void get_markers(const geometry_msgs::PolygonStamped & _data){
  x = _data.polygon.points[0].x;
  y = _data.polygon.points[0].y;
  if(!data_received){
	  std::cout << "Received the first data at " <<  x << " and " << y << std::endl;
	  data_received = true;
  }
}



int main(int argc, char * argv[]){
  //set the node
  ros::init(argc, argv, "marker_patterns");
  ros::NodeHandle nh_;
  ros::NodeHandle home("~");

  string set_no;
  string save_address;

  // get the ros params from the launch file
  home.getParam("set_no", set_no);
  home.getParam("save_address", save_address);

  cout << "set no is : " << set_no <<std::endl;
  
  int loop_freq = 7; //TODO: MAKE SURE THE NIR DATA IS ALWAYS COLLECTED AT 7 Hz
  ros::Rate loop_rate(loop_freq); 

  // define the ROS subscribers
  ros::Subscriber img_sub = nh_.subscribe("/see_scope/nir/trackers", 1,get_markers);



  std::ofstream data_file;
  try{
      //labels_file.open("/home/hsaeidi/nn_data_for_breathing_tracker/set1/labels/lebels.csv");
	data_file.open((save_address + "/" + set_no + ".csv").c_str());
  }catch (...){
      std::cout << "Could not open the data file (check the address)" <<std::endl;
  }

  while(ros::ok()){
    if(data_received){
	    data_file << x << "," << y<< std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  data_file.close();
  return 0;
}//end of main




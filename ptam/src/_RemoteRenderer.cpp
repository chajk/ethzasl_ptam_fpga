#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>

using namespace std;

class RemoteRender 
{
  ros::NodeHandle nh_, image_nh_;

public:
  
 
private:

};

/**
 * Subscriber callbacks
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "remote_render", ros::init_options::AnonymousName);
   ros::NodeHandle n;

   RemoteRender rr(n, (argc > 1) ? argv[1] : "raw");

   char key = 0;   
  
   //ros::Subscriber subCam = n.subscribe("", 10, imageCallback);
   cout <<"subscribed" << endl;

   ros::spinOnce();

   return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <iostream>
#include <gtk/gtk.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include <ptam_com/PointCloud.h>
#include <ptam_com/KeyFrame_srv.h>
#include <ptam_com/KeyFrame_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <ptam/PTAMVisualizerParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <ptam/AxesArray.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;

static void destroyNode(GtkWidget *widget, gpointer data)
{
  ros::shutdown();
}

class RemoteRender 
{
  ros::NodeHandle nh_, image_nh_;

private:
  image_transport::Subscriber *sub_;
  std::string window_name_;
  std::string transport_;
  std::string topic_;

public:
    RemoteRender(const ros::NodeHandle& nh, const std::string& _transport)
  {
     //topic_ = "vslam/preview";
     topic_ = "usb_cam/image_color";
     ros::NodeHandle local_nh("~");
     local_nh.param("window_name", window_name_, topic_);

     transport_ = _transport;

     bool autosize;
     local_nh.param("autosize", autosize, false);
     cv::namedWindow(window_name_, autosize ? 1 : 0);
     cv::resizeWindow(window_name_, 640, 480);

#ifdef HAVE_GTK
     // Register appropriate handler for when user closes the display window
     GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
     g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
#endif
     sub_ = NULL;
     subscribe(nh);
  }

  ~RemoteRender()
  {
     unsubscribe();
     cv::destroyWindow(window_name_);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
     std::cout << "imageCallback called" <<endl;
    
     cv_bridge::CvImageConstPtr cv_ptr;
     try
     {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
     {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
     }

     cv::imshow(window_name_, cv_ptr->image);
  }
 
  void subscribe(const ros::NodeHandle& nh)
  {
     if (sub_ == NULL)
     {
        image_transport::ImageTransport it(nh);
        sub_ = new image_transport::Subscriber(it.subscribe(topic_, 10, &RemoteRender::imageCallback, this, transport_));
     }
  }

  void unsubscribe()
  {
     if (sub_ != NULL)
     {
        delete sub_;
        sub_ = NULL;
     }
  } 

};

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) 
{
   std::cout <<"poseCallback called" << endl;
   
   double pos[3], attr[4];
   /**
    * Get the Position value
    */
   memcpy(pos, &(msg->pose.pose.position.x), sizeof(double)*3);
   std::cout << "pos[0]" << pos[0] << std::endl;
   std::cout << "pos[1]" << pos[1] << std::endl;
   std::cout << "pos[2]" << pos[2] << std::endl;
   //std::cout << "pos[3]" << pos[3] << std::endl;



}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "remote_render", ros::init_options::AnonymousName);
   ros::NodeHandle n;

   RemoteRender rr(n, (argc > 1) ? argv[1] : "raw");

   char key = 0;   
    
   rr.subscribe(n);
   bool subscribed = true;
  
   //ros::Subscriber subCam = n.subscribe("", 10, imageCallback);
   cout <<"subscribed" << endl;

   ros::Subscriber poseInfo = n.subscribe("vslam/pose", 10, poseCallback);
   //ros::Subscriber sub_path = n.subscribe("vslam/pose",1,&pathCallback);

   ros::Publisher key_pub = n.advertise<std_msgs::String> ("vslam/key_pressed", 10);
   std_msgs::StringPtr msg(new std_msgs::String);

   while (ros::ok())
   {
      key = cvWaitKey(10);

      if (key == ' ')
      {
         std::cout << "Sending \"Space\" to ptam" << std::endl;
         msg->data = "Space";
         key_pub.publish(msg);
      }
      else if (key == 'r')
      {
         std::cout << "Sending \"r\" to ptam" << std::endl;
         msg->data = "r";
         key_pub.publish(msg);
      }
      else if (key == 'a')
      {
         std::cout << "Sending \"a\" to ptam" << std::endl;
         msg->data = "a";
         key_pub.publish(msg);
      }
      else if (key == 'q')
      {
         std::cout << "Sending \"q\" to ptam" << std::endl;
         msg->data = "q";
         key_pub.publish(msg);
      }
      else if (key == 's')
      {
         if (subscribed)
         {
            rr.unsubscribe();
            subscribed = false;
            std::cout << "unsubscribed" << std::endl;
         }
         else
         {
            rr.subscribe(n);
            subscribed = true;
            std::cout << "subscribed" << std::endl;
         }
      }

      ros::spinOnce();

   }
 
   return 0;
}

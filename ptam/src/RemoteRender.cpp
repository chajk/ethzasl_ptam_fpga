//OpenGL_ROS. Slightly modified code from the OpenCV documentation that draws a
//cube every frame; this modified version uses the global variables rotx and roty that are
//connected to the sliders in Figure 9-6

// Note: This example needs OpenGL installed on your system. It doesn't build if 
//       the OpenGL libraries cannot be found.
#include <GL/gl.h>
#include <GL/glu.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/highgui/highgui_c.h> // Fix for build error

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <ptam_com/PointCloud.h>
#include <ptam_com/KeyFrame_srv.h>
#include <ptam_com/KeyFrame_msg.h>
#include <ptam_com/ptam_info.h>

#include <std_msgs/String.h>

#include <tf/transform_datatypes.h>
#include <cmath>

#define RADPERDEG 0.0174533

using namespace std;
using namespace cv;

float rotx = -10.0f; //25, roty = 40;
float roty = 5.0f;
float rotz = 2.0f;
float tranx = 0.0f;
float trany = 0.0f;
float tranz = -0.5f;
//int tranx = 0.0, trany=0.0, tranz = -1.0;

//Integrating model loading
unsigned int logo_faces = 0;
unsigned int spell_faces = 0;
GLfloat yrot;

GLfloat ambientLight[4];
GLfloat diffuseLight[4];
GLfloat lightPosition[4];
GLfloat specular[4];
GLfloat specref[4];

float * logo_vert_x;
float * logo_vert_y;
float * logo_vert_z;
float * logo_uv_u;
float * logo_uv_v;
float * logo_norm_x;
float * logo_norm_y;
float * logo_norm_z;

float * spell_vert_x;
float * spell_vert_y;
float * spell_vert_z;
float * spell_uv_u;
float * spell_uv_v;
float * spell_norm_x;
float * spell_norm_y;
float * spell_norm_z;

float scale = 0.15f;

double pos[3], attr[4];
double cur_posx=0.0f, cur_posy=0.0f;
double prev_posx=0.0f, prev_posy=0.0f;

bool drawGrid;
bool drawTrails;
bool mapQuality;
int trackingQuality;

bool drawObject;
unsigned int cnt = 0;

//#define M_PI 3.14159;

//string logoDir = "/home/keti/Work/catkin_ws/devel/lib/opengl_ros/KETI_LOGO.obj";
//string spellDir = "/home/keti/Work/catkin_ws/devel/lib/opengl_ros/KETI_SPELL.obj";

cv::Mat texture;
//tf::Quaternion quat;

/*
 * Method Prototype
 */
int LoadOBJ_for_KETILOGO(const char* path, float*& vert_x, float*& vert_y, float*& vert_z, float*& uv_u, float*& uv_v, float*& norm_x, float*& norm_y, float*& norm_z);
void on_opengl(void* param);
void GetNormal(GLfloat a[3], GLfloat b[3], GLfloat c[3], GLfloat normal[3]);
void on_trackbar( int, void* );
void help(char ** argv);


//Class to acquire images and camera information from ROS topic
class ImageROS
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;

   ros::Subscriber camera_info_sub;
   ros::Subscriber pose_info;
   ros::Subscriber tracker_info;

   cv::Mat rxd_img;
	
public:
   ImageROS()
    	: it_(nh_)
   {
      image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageROS::imageCb, this);
    	
      //camera_info_sub = nh_.subscribe("/usb_cam/camera_info", 1, &ImageROS::cameraInfoCb, this);
      image_pub_ = it_.advertise("/opengl_img",1);

      pose_info = nh_.subscribe("vslam/pose", 100, &ImageROS::cameraPoseCb, this);
      tracker_info = nh_.subscribe("vslam/info", 100, &ImageROS::trackerInfoCb, this);
    }

    ~ImageROS()
    {

    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        namespace enc = sensor_msgs::image_encodings;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            rxd_img = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception %s",e.what());
            return;
        }
    }

    /*void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
       cout << "cameraInfoCb called___111" << endl;
    }*/

    void cameraPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) 
    {
       //cout << "PoseCb called" << endl;
       /*
        * Get the Position value
        */
       memcpy(pos, &(msg->pose.pose.position.x), sizeof(double)*3);
       cur_posx = pos[0]; 
       cur_posy = pos[1];
       std::cout << "pos[x] " << pos[0] << std::endl;
       std::cout << "pos[y] " << pos[1] << std::endl;
       std::cout << "pos[z] " << pos[2] << std::endl;
   
       attr[0] = msg->pose.pose.orientation.w;
       memcpy(attr+1, &(msg->pose.pose.orientation.x), sizeof(double)*3);
       /*std::cout << "attr[w] " << attr[0] << std::endl;
       std::cout << "attr[x] " << attr[1] << std::endl;
       std::cout << "attr[y] " << attr[2] << std::endl;
       std::cout << "attr[z] " << attr[3] << std::endl;*/

       // TODO. Convert Quaternion to Euler
       tf::Quaternion quat(
           msg->pose.pose.orientation.x,
           msg->pose.pose.orientation.y,
           msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w );

       tf::Matrix3x3 m(quat);

       //std::cout << "qx: " << *quat << std::endl;
       //std::cout << "qy: " << *(quat+1) << std::endl;
       //std::cout << "qz: " << *(quat+2) << std::endl;
       //std::cout << "qw: " << *(quat+3) << std::endl;

       //TODO. 
       // To include glm header to convert Quaternion to Rotation Matrix.
       // To implement using glm header, it need to modify CMakeLists.txt to contain the new header files.
       double roll, pitch, yaw;
       double roll_deg, pitch_deg, yaw_deg;

       m.getRPY(roll, pitch, yaw);
       //std::cout << "roll, pitch, yaw in Radian: " << roll << ", " << pitch << ", " << yaw << std::endl;

       roll_deg = roll * 180.0 / M_PI;
       if (roll_deg < 0) roll_deg += 360.0;
       pitch_deg = pitch * 180.0 / M_PI;
       if (pitch_deg < 0) pitch_deg += 360.0;
       yaw_deg = yaw * 180.0 / M_PI;
       if (yaw_deg < 0) yaw_deg += 360.0;
       
       std::cout << "roll, pitch, yaw in Degree: " << roll_deg << ", " << pitch_deg << ", " << yaw_deg << std::endl;

       /*
       double r, p, y;
       double r_deg, p_deg, y_deg; 
       double qx, qy, qz, qw;
       qx = *quat; qy = *(quat+1); qz = *(quat+2); qw = *(quat+3);
       
       //r = atan2f(2.0f * (qw*qz + qx*qy), 1 - 2*(qz*qz+qx*qx));
       r = roll;
       r_deg = r * 180.0 / M_PI; 

       p = pitch; 
       p_deg = p * 180.0 / M_PI;

       y = yaw;
       y_deg = y * 180.0 / M_PI;

       std::cout << "r: " << r << std::endl;
       
       std::cout << "r_deg: " << r_deg << std::endl;
       std::cout << "p_deg: " << p_deg << std::endl;
       std::cout << "y_deg: " << y_deg << std::endl;
       */

       //For Testing
       //rotx = static_cast<float>(roll);
       //roty = static_cast<float>(pitch);
       //rotz = static_cast<float>(yaw);
 
       //TODO. 
       // Simply determine 2D position using given vector. 
       // After simple positioning, It also need to implement GL Rotation matrix by converting Quaternion. When trying include glm header into project, I couldn't make success in time. 
       // Pos[x] : Camera moving right (-), moving left (+)
       // Pos[y] : Camera moving upward (+), moving downward (-)
       double deltax, deltay;
       deltax = cur_posx - prev_posx;
       deltay = cur_posy - prev_posy;

       // Rotate Y-axis (Yaw)
       if (cur_posx - prev_posx < -0.005 ) { //Camera moving right. "-" incremental
          roty += std::abs (deltax) * 30;
          tranx -= std::abs(deltax) * 2; // Object moves left.
          //std::cout << "Moving right, prev:cur " << prev_posx << ", " << cur_posx << ", " << deltax << std::endl;
       } else if (cur_posx - prev_posx > 0.005 ) { //Camera moving left. "+" incremental
          roty -= std::abs (deltax) * 30;
          tranx += std::abs(deltax) * 2; // Object moves right
          //std::cout << "Moving left: " << prev_posx << ", " << cur_posx << ", " << deltax << std::endl;
       } else {
          //std::cout << "Delta of L/R Moving: " << prev_posx << ", " << cur_posx << ", " << deltax << std::endl;
       }
       // Rotate X-axis (Roll)
       if (cur_posy - prev_posy > 0.005 ) { //Camera moving upward. 
          rotx -= std::abs (deltay) * 30;
          trany -= std::abs (deltay) * 3;
          //std::cout << "Up: prev:cur " << deltay << std::endl; //prev_posy << ", " << cur_posy << ", " << deltay << std::endl; 
       } else if (deltay < -0.005 ) { //Camera moving downward.
          rotx += std::abs (deltay) * 30;
          trany += std::abs (deltay) * 3;
          //std::cout << "Down: prev:cur " << deltay << std::endl; 
       } else {
          //std::cout << "Delta of Up/Down moving: " << prev_posy << ", " << cur_posy << ", " << deltay << std::endl;
       }
       prev_posx = cur_posx;
       prev_posy = cur_posy;


       //TODO. Determine the scale depending on position[z].
       // range 0.005(smallest) ~ 0.2 (largest)
       scale = 0.2f - pos[2] * 0.2;

       std::cout << "scale: " << scale <<  std::endl;
       std::cout << "tranx: " << tranx << std::endl;
       std::cout << "trany: " << trany << std::endl;
       //std::cout << "rotx: " << rotx <<  std::endl;
       //std::cout << "roty: " << roty <<  std::endl;
       //std::cout << "rotz: " << rotz <<  std::endl;
      
    }

    void trackerInfoCb(const ptam_com::ptam_infoPtr & msg)
    {
       //cout << "trackerInfoCb called___333" << endl;

       //TODO, Get the trails container
       //std::list<Trail> & trails = mpTracker->getTrails();

       //drawGrid = msg->drawGrid;
       //std::cout << "drawGrid: " << drawGrid << std::endl;

       //drawTrails = msg->drawTrails;
       //std::cout << "drawTrails: " << drawTrails << std::endl;
 
       mapQuality = msg->mapQuality;
       //std::cout << "mapQuality: " << mapQuality << std::endl;

       trackingQuality = msg->trackingQuality;
       //std::cout << "trackingQuality: " << trackingQuality << std::endl;

#if 0 // Setting for Desktop latency      
       // TODO. For smooth appearence
       if (trackingQuality == 2 && cnt < 10) 
       {
          cnt += 1; 
          cout << "cnt added"<< endl;
       } 
       if (trackingQuality == 0 && cnt > 0 && cnt < 11) 
       {
          cnt -= 1;
          cout << "cnt subtracted" << endl;   
       }
       if (cnt == 10) 
       {
          drawObject = true;
          cout << "drawObject == true" << endl;
       } else if (cnt == 0) {
          drawObject = false;
          cout << "drawObject == false" << endl;
       }
#else  // Setting for Stratix10 latency
       // TODO. For smooth appearence
       if (trackingQuality == 2) 
       {
          //cnt ++;
          cnt = 3;
          drawObject = true; 
          //cout << "drawObject == true" << endl;
       } 
       if (trackingQuality == 0 && cnt > 0 && cnt < 5) 
       {
          cnt --;
          //cout << "cnt subtracted" << endl;   
       }
       if (cnt == 0) 
       {
          drawObject = false;
          //cout << "drawObject == false" << endl;
       }
       //if (cnt > 0) 
       //{
       //   drawObject = true;
       //   cnt = 3;
       //   cout << "drawObject == true" << endl;
       //} else if (cnt == 0) {
       //   drawObject = false;
       //   cout << "drawObject == false" << endl;
       //}

#endif

       //cout << "cnt: " << cnt << endl;
    }

    cv::Mat getImage(void)
    {
        cv::Mat test_img;
        if(!rxd_img.empty())
        {
            return rxd_img;
        }
        else
        {
            return test_img;
        }
    }

    //TODO
    void publishGLImage(cv::Mat gl_img)
    {
        if(!gl_img.empty())
        {

            cv_bridge::CvImage out_msg;
            //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
            out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
            out_msg.image    = gl_img; // Your cv::Mat

            image_pub_.publish(out_msg.toImageMsg());

        }
    }

};

int main(int argc, char* argv[])
{

   ros::init(argc,argv,"opengl_node");
   ros::NodeHandle n;
    
   ImageROS obj;
   cv::Mat img;     

   ros::Publisher key_pub = n.advertise<std_msgs::String> ("vslam/key_pressed", 10);
   std_msgs::StringPtr msg(new std_msgs::String);
 
   // Added for KETI LOGO Loading
   ambientLight[0] = 1.0f;
   ambientLight[1] = 1.0f;
   ambientLight[2] = 1.0f;
   ambientLight[3] = 1.0f;

   diffuseLight[0] = 0.7f;
   diffuseLight[1] = 0.7f;
   diffuseLight[2] = 0.7f;
   diffuseLight[3] = 1.0f;

   lightPosition[0] = 0.0f;
   lightPosition[1] = 0.0f;
   lightPosition[2] = 0.0f;
   lightPosition[3] = 1.0f;

   specular[0] = 1.0f;
   specular[1] = 1.0f;
   specular[2] = 1.0f;
   specular[3] = 1.0f;

   yrot = 0.0f;

   logo_faces = LoadOBJ_for_KETILOGO("/home/keti/Work/catkin_ws/devel/lib/opengl_ros/KETI_LOGO.obj", 
                                      logo_vert_x, logo_vert_y, logo_vert_z, 
                                      logo_uv_u, logo_uv_v, 
                                      logo_norm_x, logo_norm_y, logo_norm_z
                                    );

   spell_faces = LoadOBJ_for_KETILOGO( "/home/keti/Work/catkin_ws/devel/lib/opengl_ros/KETI_SPELL.obj", 
                                       spell_vert_x, spell_vert_y, spell_vert_z, 
                                       spell_uv_u, spell_uv_v, 
                                       spell_norm_x, spell_norm_y, spell_norm_z
                                     );

   glEnable(GL_DEPTH_TEST);
   glEnable(GL_CULL_FACE);

   ros::Rate r(30); // 30 hz

   while(img.empty())
   {
       img = obj.getImage();
       ros::spinOnce();
       ROS_INFO("Waiting for images");
       ros::Duration(0.01).sleep(); 
   }
 
   if (img.empty()) {
      cout << "Cannot load image: " << endl;
      exit(EXIT_FAILURE);
   }    

   cv::namedWindow( "OpenGL_ROS", CV_WINDOW_OPENGL );
   cv::resizeWindow("OpenGL_ROS", img.cols, img.rows);
   cv::moveWindow("OpenGL_ROS", 700, 50);
   
   /*cv::createTrackbar( "X-rot", "OpenGL_ROS", &rotx, 360, on_trackbar);
   cv::createTrackbar( "Y-rot", "OpenGL_ROS", &roty, 360, on_trackbar);
   cv::createTrackbar( "X-tran", "OpenGL_ROS", &tranx, 10, on_trackbar);
   cv::createTrackbar( "Y-tran", "OpenGL_ROS", &trany, 10, on_trackbar);
   cv::createTrackbar( "Z-tran", "OpenGL_ROS", &tranz, 10, on_trackbar);*/
  
   string text = "No Map saved";
   int fontFace = FONT_HERSHEY_SIMPLEX;
   double fontScale = 0.5;
   int thickness = 1.0;
   Point org(30, 450);
 
   while(ros::ok())
   {
        if (mapQuality == 1 && trackingQuality == 2) {
           text = "Map found";
        } else if (mapQuality == 1 && trackingQuality == 0) { 
           text = "Searching Map";
        }

        //std::cout << "scale: " << scale <<  std::endl;

   	img = obj.getImage();

        cv::putText(img, text, org, fontFace, fontScale, Scalar(0,255,255), thickness);

   	cv::ogl::Texture2D backgroundTex(img,true);   
   	cv::setOpenGlDrawCallback("OpenGL_ROS", on_opengl, &backgroundTex);
   	cv::updateWindow("OpenGL_ROS");
        ros::spinOnce(); 
  
        int key = cv::waitKey(10);	
        if (key != -1) cout << "key pressed: " << key << endl;

        if (key == 27) {
           break; // if escape pressed then break
        } 
        else if (key == 32) { // space key
           std::cout << "Sending \"Space\" to ptam" << std::endl;
           msg->data = "Space";
           key_pub.publish(msg);
           text = "Translate camera(10cm) to make a Map";
        }
        else if (key == 114) { // r key
           std::cout << "Sending \"r\" to ptam" << std::endl;
           msg->data = "r";
           key_pub.publish(msg); 
           text = "Canceled, Try to make a Map";
        }
        else if (key == 61) { // + key
           rotx += 1.0;
        } 
        else if (key == 45) { // - key
           rotx -= 1.0;
        } 
        else if (key == 93) { // } key
           roty += 1.0;
        } 
        else if (key == 91) { // { key
           roty -= 1.0;
        }  
        else if (key == 39) { // ' key
           rotz += 1.0;
        } 
        else if (key == 59) { // ; key
           rotz -= 1.0;
        } 
        else if (key == 119) { // w key 
           tranx += 0.025;
        } 
        else if (key == 113) { // q key
           tranx -= 0.025;
        }
        else if (key == 115) { // s sky
           trany += 0.025;
        } 
        else if (key == 97) { // a key
           trany -= 0.025;
        } 
        else if (key == 120) { // x key
           tranz += 0.025;
        } 
        else if (key == 122) { // z key
           tranz -= 0.025;
        } 
        else if (key == 50) { // 2 key
           scale += 0.005;
           //std::cout << "cur scale(+): " << scale << std::endl;
        } 
        else if (key == 49) { // 1 key
           scale -= 0.005;
           //std::cout << "cur scale(-): " << scale << std::endl;
        }
	backgroundTex.release(); //Releasing the background texture to avoid memory leak
	
   }
   cv::setOpenGlDrawCallback("OpenGL_ROS", 0, 0);
   cv::destroyAllWindows();

   free(logo_vert_x);
   free(logo_vert_y);
   free(logo_vert_z);

   free(logo_uv_u);
   free(logo_uv_v);
   free(logo_norm_x);
   free(logo_norm_y);
   free(logo_norm_z);

   free(spell_vert_x);
   free(spell_vert_y);
   free(spell_vert_z);

   free(spell_uv_u);
   free(spell_uv_v);
   free(spell_norm_x);
   free(spell_norm_y);
   free(spell_norm_z);

   return 0;
}

int LoadOBJ_for_KETILOGO(const char* path, float*& vert_x, float*& vert_y, float*& vert_z, float*& uv_u, float*& uv_v, float*& norm_x, float*& norm_y, float*& norm_z)
{
   float _temp_vert_x[1000];
   float _temp_vert_y[1000];
   float _temp_vert_z[1000];
   unsigned int vert_cnt = 0;

   float _temp_uv_u[1000];
   float _temp_uv_v[1000];
   unsigned int uv_cnt = 0;

   float _temp_norm_x[1000];
   float _temp_norm_y[1000];
   float _temp_norm_z[1000];
   unsigned int norm_cnt = 0;

   unsigned int _temp_face_vert[10000];
   unsigned int _temp_face_uv[10000];
   unsigned int _temp_face_norm[10000];
   unsigned int face_cnt = 0;

   float single_vert_x = 0.0f;
   float single_vert_y = 0.0f;
   float single_vert_z = 0.0f;
   float single_uv_u = 0.0f;
   float single_uv_v = 0.0f;
   float single_norm_x = 0.0f;
   float single_norm_y = 0.0f;
   float single_norm_z = 0.0f;

   unsigned int face_1st_vert = 0;
   unsigned int face_2nd_vert = 0;
   unsigned int face_3rd_vert = 0;

   unsigned int face_1st_uv = 0;
   unsigned int face_2nd_uv = 0;
   unsigned int face_3rd_uv = 0;

   unsigned int face_1st_norm = 0;
   unsigned int face_2nd_norm = 0;
   unsigned int face_3rd_norm = 0;

   FILE* file = fopen(path, "r");
   if (file == NULL)
   {
      cout << "Failed to open file" << endl;
      getchar();
      return -1;
   } else {
      cout << "Success to open file: " << path << endl;
   }

   while (1)
   {
      char lineHeader[128];
      int res = fscanf(file, "%s", lineHeader);

      if (res == EOF) 
         break; 

      // temp °ø°£¿¡ vertex, uv, normal ÀúÀå
      if (strcmp(lineHeader, "v") == 0)
      {
         fscanf(file, "%f %f %f\n", &single_vert_x, &single_vert_y, &single_vert_z);
         _temp_vert_x[vert_cnt] = single_vert_x;
	 _temp_vert_y[vert_cnt] = single_vert_y;
	 _temp_vert_z[vert_cnt] = single_vert_z;
	 vert_cnt++;
      }
      else if (strcmp(lineHeader, "vt") == 0)
      {
         fscanf(file, "%f %f\n", &single_uv_u, &single_uv_v);
         _temp_uv_u[uv_cnt] = single_uv_u;
         _temp_uv_v[uv_cnt] = single_uv_v;
         uv_cnt++;
      }
      else if (strcmp(lineHeader, "vn") == 0)
      {
         fscanf(file, "%f %f %f\n", &single_norm_x, &single_norm_y, &single_norm_z);
         _temp_norm_x[norm_cnt] = single_norm_x;
         _temp_norm_y[norm_cnt] = single_norm_y;
         _temp_norm_z[norm_cnt] = single_norm_z;
         norm_cnt++;
      }
      else if (strcmp(lineHeader, "f") == 0)
      {
         fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", 
                &face_1st_vert, &face_1st_uv, 
                &face_1st_norm, &face_2nd_vert, 
                &face_2nd_uv, &face_2nd_norm, 
                &face_3rd_vert, &face_3rd_uv, 
                &face_3rd_norm
               );
         _temp_face_vert[face_cnt * 3 + 0] = face_1st_vert;
         _temp_face_vert[face_cnt * 3 + 1] = face_2nd_vert;
         _temp_face_vert[face_cnt * 3 + 2] = face_3rd_vert;
         _temp_face_uv[face_cnt * 3 + 0] = face_1st_uv;
         _temp_face_uv[face_cnt * 3 + 1] = face_2nd_uv;
         _temp_face_uv[face_cnt * 3 + 2] = face_3rd_uv;
         _temp_face_norm[face_cnt * 3 + 0] = face_1st_norm;
         _temp_face_norm[face_cnt * 3 + 1] = face_2nd_norm;
         _temp_face_norm[face_cnt * 3 + 2] = face_3rd_norm;
         face_cnt++;
      }
   }

   //face_cnt--;
   // face Ä«¿îÆ® žžÅ­, ž®ÅÏ µÇŽÂ À§ÀÇ ¹è¿­ µ¿ÀûÇÒŽç ¹× ŒøŒ­Žë·Î Ã€¿ì±â
   vert_x = (float*)malloc(sizeof(float) * (face_cnt * 3));
   vert_y = (float*)malloc(sizeof(float) * (face_cnt * 3));
   vert_z = (float*)malloc(sizeof(float) * (face_cnt * 3));

   uv_u = (float*)malloc(sizeof(float) * (face_cnt * 3));
   uv_v = (float*)malloc(sizeof(float) * (face_cnt * 3));

   norm_x = (float*)malloc(sizeof(float) * (face_cnt * 3));
   norm_y = (float*)malloc(sizeof(float) * (face_cnt * 3));
   norm_z = (float*)malloc(sizeof(float) * (face_cnt * 3));

   for (int i = 0; i < face_cnt; i++)
   {
      vert_x[i * 3 + 0] = _temp_vert_x[_temp_face_vert[i * 3 + 0] - 1];
      vert_x[i * 3 + 1] = _temp_vert_x[_temp_face_vert[i * 3 + 1] - 1];
      vert_x[i * 3 + 2] = _temp_vert_x[_temp_face_vert[i * 3 + 2] - 1];

      vert_y[i * 3 + 0] = _temp_vert_y[_temp_face_vert[i * 3 + 0] - 1];
      vert_y[i * 3 + 1] = _temp_vert_y[_temp_face_vert[i * 3 + 1] - 1];
      vert_y[i * 3 + 2] = _temp_vert_y[_temp_face_vert[i * 3 + 2] - 1];

      vert_z[i * 3 + 0] = _temp_vert_z[_temp_face_vert[i * 3 + 0] - 1];
      vert_z[i * 3 + 1] = _temp_vert_z[_temp_face_vert[i * 3 + 1] - 1];
      vert_z[i * 3 + 2] = _temp_vert_z[_temp_face_vert[i * 3 + 2] - 1];

      uv_u[i * 3 + 0] = _temp_uv_u[_temp_face_uv[i * 3 + 0] - 1];
      uv_u[i * 3 + 1] = _temp_uv_u[_temp_face_uv[i * 3 + 1] - 1];
      uv_u[i * 3 + 2] = _temp_uv_u[_temp_face_uv[i * 3 + 2] - 1];

      uv_v[i * 3 + 0] = _temp_uv_u[_temp_face_uv[i * 3 + 0] - 1];
      uv_v[i * 3 + 1] = _temp_uv_u[_temp_face_uv[i * 3 + 1] - 1];
      uv_v[i * 3 + 2] = _temp_uv_u[_temp_face_uv[i * 3 + 2] - 1];

      norm_x[i * 3 + 0] = _temp_norm_x[_temp_face_norm[i * 3 + 0] - 1];
      norm_x[i * 3 + 1] = _temp_norm_x[_temp_face_norm[i * 3 + 1] - 1];
      norm_x[i * 3 + 2] = _temp_norm_x[_temp_face_norm[i * 3 + 2] - 1];

      norm_y[i * 3 + 0] = _temp_norm_y[_temp_face_norm[i * 3 + 0] - 1];
      norm_y[i * 3 + 1] = _temp_norm_y[_temp_face_norm[i * 3 + 1] - 1];
      norm_y[i * 3 + 2] = _temp_norm_y[_temp_face_norm[i * 3 + 2] - 1];

      norm_z[i * 3 + 0] = _temp_norm_z[_temp_face_norm[i * 3 + 0] - 1];
      norm_z[i * 3 + 1] = _temp_norm_z[_temp_face_norm[i * 3 + 1] - 1];
      norm_z[i * 3 + 2] = _temp_norm_z[_temp_face_norm[i * 3 + 2] - 1];
   }
   return face_cnt;
} // End of LoadOBJ for KETI Logo

void on_opengl(void* param) 
{
    //cout << "on_opengl" << endl;
    
    cv::ogl::Texture2D* backgroundTex = (cv::ogl::Texture2D*)param;
    glEnable( GL_TEXTURE_2D );
    backgroundTex->bind();
    cv::ogl::render(*backgroundTex);
    glDisable( GL_TEXTURE_2D );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    
    //glTranslatef(0, 0, -0.5);
    glTranslatef(tranx, trany, tranz);
    //glTranslatef(0, 0, -0.5);
 
    //glRotatef( rotx, 1, 0, 0 );
    //glRotatef( roty, 0, 1, 0 );
    //glRotatef( 0, 0, 0, 1 );
    //glRotatef( rotz, 0, 0, 1 );
 
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LESS );
    
    if (mapQuality == 1 && drawObject == true) //trackingQuality !=0)  
    {
#if 1
/*
    // Added for model loading
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glEnable(GL_LIGHT0);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // ------------------ LOGO --------------------------
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 5.0f);
    glRotatef(yrot, 0.0f, 1.0f, 0.0f);
    glColor3f(0.078f, 0.224f, 0.302f);
    glBegin(GL_TRIANGLES);
       for (int face_i = 0; face_i < logo_faces; face_i = face_i + 3)
       {
          glNormal3f(logo_norm_x[face_i], logo_norm_y[face_i], logo_norm_z[face_i]);
	  //glVertex3f(logo_vert_x[face_i + 0] * scale, logo_vert_y[face_i + 0] * scale, logo_vert_z[face_i + 0] * scale);
          glVertex3f(logo_vert_x[face_i + 0] * scale, logo_vert_y[face_i + 0] * scale, logo_vert_z[face_i + 0] * scale);
	  glVertex3f(logo_vert_x[face_i + 1] * scale, logo_vert_y[face_i + 1] * scale, logo_vert_z[face_i + 1] * scale);
	  glVertex3f(logo_vert_x[face_i + 2] * scale, logo_vert_y[face_i + 2] * scale, logo_vert_z[face_i + 2] * scale);
       }
       glEnd();
       glPopMatrix();
   // ------------------ LOGO END --------------------

   // ------------------ SPELL --------------------------
   glPushMatrix();
   glTranslatef(0.0f, 0.0f, -5.0f);
   glRotatef(yrot, 0.0f, 1.0f, 0.0f);
   glColor3f(0.012f, 0.071f, 0.145f);
   glBegin(GL_TRIANGLES);
      for (int face_i = 0; face_i < spell_faces; face_i = face_i + 3)
      {
         glNormal3f(spell_norm_x[face_i], spell_norm_y[face_i], spell_norm_z[face_i]);
	 glVertex3d(spell_vert_x[face_i + 0] * scale, spell_vert_y[face_i + 0] * scale, spell_vert_z[face_i + 0] * scale);
	 glVertex3d(spell_vert_x[face_i + 1] * scale, spell_vert_y[face_i + 1] * scale, spell_vert_z[face_i + 1] * scale);
	 glVertex3d(spell_vert_x[face_i + 2] * scale, spell_vert_y[face_i + 2] * scale, spell_vert_z[face_i + 2] * scale);
      }
      glEnd();
      glPopMatrix();
   // ------------------ SPELL END --------------------
   yrot += 1.0f;
   glFlush();
*/
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	//glCullFace(GL_FRONT_FACE);
	
	glShadeModel(GL_FLAT);
	glFrontFace(GL_CCW);
	
	GLfloat lightPower = 5.0f;
	GLfloat spectPower = 10.0f;

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat ambient[] = { lightPower, lightPower, lightPower, 1.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	GLfloat diffuse[] = { lightPower, lightPower, lightPower, 1.0f };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	GLfloat spec[] = { spectPower, spectPower, spectPower, 1.0f };
	glLightfv(GL_LIGHT0, GL_SPECULAR, ambient);
	GLfloat position[] = { 3.0, 3.0, 1.0, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glEnable(GL_COLOR_MATERIAL);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glPushMatrix();
	glRotatef(rotx/*xAngle*/, 1.0f, 0.0f, 0.0f);
	glRotatef(roty/*yAngle*/, 0.0f, 1.0f, 0.0f);
	glRotatef(rotz/*zAngle*/, 0.0f, 0.0f, 1.0f);
	
	glBegin(GL_TRIANGLES);
	glColor3f(0.078f, 0.224f, 0.302f);

	GLfloat normal[3];

	for (int face_i = 0; face_i < logo_faces; face_i ++)
	{
		GLfloat Triangle_Logo[3][3] = {
			{logo_vert_x[face_i * 3 + 0] * scale, logo_vert_y[face_i * 3 + 0] * scale, logo_vert_z[face_i * 3 + 0] * scale},
			{logo_vert_x[face_i * 3 + 1] * scale, logo_vert_y[face_i * 3 + 1] * scale, logo_vert_z[face_i * 3 + 1] * scale},
			{logo_vert_x[face_i * 3 + 2] * scale, logo_vert_y[face_i * 3 + 2] * scale, logo_vert_z[face_i * 3 + 2] * scale},
		};
// 		GetNormal(Triangle_Logo[0], Triangle_Logo[1], Triangle_Logo[2], normal);
// 		if (bNormal) { glNormal3fv(normal); }
		glNormal3f(logo_norm_x[face_i * 3], logo_norm_y[face_i * 3], logo_norm_z[face_i * 3]);
		glVertex3fv(Triangle_Logo[2]);
		glVertex3fv(Triangle_Logo[1]);
		glVertex3fv(Triangle_Logo[0]);
	}
	
	glColor3f(0.012f, 0.071f, 0.145f);
	for (int face_i = 0; face_i < spell_faces; face_i++)
	{
		GLfloat Triangle_Spell[3][3] = {
			{spell_vert_x[face_i * 3 + 0] * scale, spell_vert_y[face_i * 3 + 0] * scale, spell_vert_z[face_i * 3 + 0] * scale},
			{spell_vert_x[face_i * 3 + 1] * scale, spell_vert_y[face_i * 3 + 1] * scale, spell_vert_z[face_i * 3 + 1] * scale},
			{spell_vert_x[face_i * 3 + 2] * scale, spell_vert_y[face_i * 3 + 2] * scale, spell_vert_z[face_i * 3 + 2] * scale},
		};
// 		GetNormal(Triangle_Spell[0], Triangle_Spell[1], Triangle_Spell[2], normal);
// 		if (bNormal) { glNormal3fv(normal); }
		glNormal3f(spell_norm_x[face_i * 3], spell_norm_y[face_i * 3], spell_norm_z[face_i * 3]);
		glVertex3fv(Triangle_Spell[2]);
		glVertex3fv(Triangle_Spell[1]);
		glVertex3fv(Triangle_Spell[0]);
	}

	glEnd();
	glPopMatrix();
	glFlush();
	yrot++;

#else
/*
   static const int coords[6][4][3] = {
        { { +1, -1, -1 }, { -1, -1, -1 }, { -1, +1, -1 }, { +1, +1, -1 } },
        { { +1, +1, -1 }, { -1, +1, -1 }, { -1, +1, +1 }, { +1, +1, +1 } },
        { { +1, -1, +1 }, { +1, -1, -1 }, { +1, +1, -1 }, { +1, +1, +1 } },
        { { -1, -1, -1 }, { -1, -1, +1 }, { -1, +1, +1 }, { -1, +1, -1 } },
        { { +1, -1, +1 }, { -1, -1, +1 }, { -1, -1, -1 }, { +1, -1, -1 } },
        { { -1, -1, +1 }, { +1, -1, +1 }, { +1, +1, +1 }, { -1, +1, +1 } }
   };

   for (int i = 0; i < 6; ++i) {
       glColor3ub( i*20, 100+i*10, i*42 );
       glBegin( GL_QUADS );
       for (int j = 0; j < 4; ++j) {
           glVertex3d(
                       0.3 * coords[i][j][0],
                       0.3 * coords[i][j][1],
                       0.3 * coords[i][j][2]
                     );
       }
       glEnd();
   }
*/
 
#endif
    } 
    //mapQuality = 0;
    //trackingQuality = 0;
} // End of on_opengl

void GetNormal(GLfloat a[3], GLfloat b[3], GLfloat c[3], GLfloat normal[3])
{
	GLfloat ba[3];
	GLfloat ca[3];
	GLfloat n[3];

	ba[0] = b[0] - a[0]; ba[1] = b[1] - a[1]; ba[2] = b[2] - a[2];
	ca[0] = c[0] - a[0]; ca[1] = c[1] - a[1]; ca[2] = c[2] - a[2];

	n[0] = ba[1] * ca[2] - ca[1] * ba[2];
	n[1] = ca[0] * ba[2] - ba[0] * ca[2];
	n[2] = ba[0] * ca[1] - ca[0] * ba[1];

	GLfloat l = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	normal[0] = n[0] / l; normal[1] = n[1] / l; normal[2] = n[2] / l;
}

void on_trackbar( int, void* ) {
    cv::updateWindow( "OpenGL_ROS" );
}

void help(char ** argv) {
	
	cout << "\n//OpenGL_ROS. Slightly modified code from the OpenCV documentation that draws a"
	     << "\n//cube every frame; this modified version uses the global variables rotx and roty that are"
	     << "\n//connected to the sliders in Figure 9-6"
	     << "\n// Note: This example needs OpenGL installed on your system. It doesn't build if" 
	     << "\n//       the OpenGL libraries cannot be found.\n"
	     << "\nCall: " << argv[0] << " <image>\n\n"
	     << "\nHere OpenGL is used to render a cube on top of an image.\n"
             << "\nUser can rotate the cube with the sliders\n" <<endl;
}


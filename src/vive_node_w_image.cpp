#include <cmath>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
//
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
//
#include "vive_ros/vr_interface.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std;

//void handleDebugMessages(const std::string &msg) {ROS_DEBUG(" [VIVE] %s",msg.c_str());}
//void handleInfoMessages(const std::string &msg) {ROS_INFO(" [VIVE] %s",msg.c_str());}
//void handleErrorMessages(const std::string &msg) {ROS_ERROR(" [VIVE] %s",msg.c_str());}
void handleDebugMessages(const std::string &msg) { std::cerr << "[VIVE] debg : " << msg << std::endl; }
void handleInfoMessages(const std::string &msg)  { std::cerr << "[VIVE] info : " << msg << std::endl; }
void handleErrorMessages(const std::string &msg) { std::cerr << "[VIVE] erro : " << msg << std::endl; }
void mySigintHandler(int sig){
// Do some custom action.
// For example, publish a stop message to some other nodes.
// All the default sigint handler does is call shutdown()
//ros::shutdown();
}

//#define PUBLISH_TWIST
#define USE_IMAGE

#define USE_OPENGL
//#define USE_VULKAN

#ifdef USE_IMAGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/camera_info.h>
#include <cv_bridge/cv_bridge.h>
enum {X, Y, XY};
enum {L, R, LR};

//#if defined USE_OPENGL
#include "vive_ros/hellovr_opengl_main.h"
class CMainApplicationMod : public CMainApplication{
  public:
    CMainApplicationMod( int argc, char *argv[] )
    : CMainApplication( argc, argv )
    , hmd_fov(110*M_PI/180) {
//      m_bShowCubes = false;
      for(int i=0;i<LR;i++){
        cam_f[i][X] = cam_f[i][Y] = 600;
      }
      RenderFrame_hz_count = 0;
    };
    ~CMainApplicationMod(){};
    VRInterface* vr_p;

    cv::Mat ros_img[LR];
    double cam_f[LR][XY];
    const double hmd_fov;//field of view
    float hmd_fov_h, hmd_fov_v;
    int RenderFrame_hz_count;

    void InitTextures(){
        std::cout << "Render " << m_nRenderWidth << " x " << m_nRenderHeight << std::endl;
      ros_img[L] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(255,0,0));
      ros_img[R] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(0,255,0));
      hmd_panel_img[L] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      hmd_panel_img[R] = cv::Mat(cv::Size(m_nRenderWidth, m_nRenderHeight), CV_8UC3, CV_RGB(100,100,100));
      for ( int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++){
        if(m_pHMD->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD){
          m_pHMD->GetStringTrackedDeviceProperty( i, vr::Prop_ScreenshotHorizontalFieldOfViewDegrees_Float, (char *)&hmd_fov_h, sizeof(float), NULL );
          m_pHMD->GetStringTrackedDeviceProperty( i, vr::Prop_ScreenshotVerticalFieldOfViewDegrees_Float, (char *)&hmd_fov_v, sizeof(float), NULL );
        }
      }
    }
    void RenderFrame(){
      //ros::Time tmp = ros::Time::now();
      //std::cout << "Rf(in)" << std::endl;
      if ( m_pHMD ){
        //std::cout << "Rf(HMD)" << std::endl;
        RenderControllerAxes();
        RenderStereoTargets();
        //std::cout << "Rf(update_texture)" << std::endl;
        UpdateTexturemaps();
        RenderCompanionWindow();
        vr::Texture_t leftEyeTexture = {(void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture );
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
      }
      if ( m_bVblank && m_bGlFinishHack ){ glFinish(); }
      SDL_GL_SwapWindow( m_pCompanionWindow );
      glClearColor( 0, 0, 0, 1 );
      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
      if ( m_bVblank ){
        glFlush();
        glFinish();
      }
      if ( m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last ){
        m_iValidPoseCount_Last = m_iValidPoseCount;
        m_iTrackedControllerCount_Last = m_iTrackedControllerCount;
        //dprintf( "PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount );
      }
      UpdateHMDMatrixPose();
      //ROS_INFO_THROTTLE(3.0,"RenderFrame() @ %d [fps]", [](int& cin, int dur){int ans = cin; cin=0; return ans/dur;}(RenderFrame_hz_count, 3));
      RenderFrame_hz_count++;
    }

  private:
    cv::Mat hmd_panel_img[LR];
    cv::Mat ros_img_resized[LR];
    void processROSStereoImage(cv::Mat (&in)[LR], cv::Mat (&out)[LR])
    {
        // just resize
        for(int i=0;i<LR;i++){
            cv::Mat tmp;
            cv::resize(in[i], tmp, cv::Size(out[i].cols, out[i].rows));
            cv::flip(tmp, tmp, 0);
            tmp.copyTo(out[i]);
        }
    }
#if 0
    void processROSStereoImage(cv::Mat (&in)[LR], cv::Mat (&out)[LR]){ // in:ros_img, out:hmd_panel_img
      const double hmd_eye2panel_z[XY] = { (double)out[L].cols/2/tan(hmd_fov/2), (double)out[L].rows/2/tan(hmd_fov/2) };
      const double cam_pic_size[LR][XY] = { { (double)in[L].cols, (double)in[L].rows }, { (double)in[R].cols, (double)in[R].rows } };
      double cam_fov[LR][XY];
      int cam_pic_size_on_hmd[LR][XY];
      cv::Mat hmd_panel_roi[LR];
      for(int i=0;i<LR;i++){
        //ROS_INFO_THROTTLE(3.0,"Process ROS image[%d] (%dx%d) with fov (%dx%d) to (%dx%d)", i, in[i].cols, in[i].rows, (int)cam_f[i][X], (int)cam_f[i][Y], out[i].cols, out[i].rows);
        for(int j=0;j<XY;j++){
          cam_fov[i][j] = 2 * atan( cam_pic_size[i][j]/2 / cam_f[i][j] );
          cam_pic_size_on_hmd[i][j] = (int)( hmd_eye2panel_z[j] * 2 * tan(cam_fov[i][j]/2) );
        }
        cv::resize(in[i], ros_img_resized[i], cv::Size(cam_pic_size_on_hmd[i][X], cam_pic_size_on_hmd[i][Y]));
        cv::flip(ros_img_resized[i], ros_img_resized[i], 0);
        cv::Rect  hmd_panel_area_rect( ros_img_resized[i].cols/2 - out[i].cols/2,
                                       ros_img_resized[i].rows/2 - out[i].rows/2,
                                       out[i].cols, out[i].rows);
        cv::Rect  ros_img_resized_rect( 0, 0, ros_img_resized[i].cols, ros_img_resized[i].rows);
        cv::Point ros_img_resized_center(ros_img_resized[i].cols/2, ros_img_resized[i].rows/2);
        cv::Rect cropped_rect;
        if( !hmd_panel_area_rect.contains( cv::Point(ros_img_resized_rect.x, ros_img_resized_rect.y) )
            || !hmd_panel_area_rect.contains( cv::Point(ros_img_resized_rect.x+ros_img_resized_rect.width,ros_img_resized_rect.y+ros_img_resized_rect.height) ) ){
          //ROS_WARN_THROTTLE(3.0,"Resized ROS image[%d] (%dx%d) exceed HMD eye texture (%dx%d) -> Cropping",i,cam_pic_size_on_hmd[i][X],cam_pic_size_on_hmd[i][Y],m_nRenderWidth,m_nRenderHeight);
          cropped_rect = ros_img_resized_rect & hmd_panel_area_rect;
          ros_img_resized[i] = ros_img_resized[i](cropped_rect);
        }
        cv::Rect hmd_panel_draw_rect( cropped_rect.x-hmd_panel_area_rect.x, cropped_rect.y-hmd_panel_area_rect.y, ros_img_resized[i].cols, ros_img_resized[i].rows);
        ros_img_resized[i].copyTo(out[i](hmd_panel_draw_rect));
      }
    }
#endif

    void UpdateTexturemaps(){
      processROSStereoImage(ros_img, hmd_panel_img);
      for(int i=0;i<LR;i++){
        if(i==L)glBindTexture( GL_TEXTURE_2D, leftEyeDesc.m_nResolveTextureId );
        else if(i==R)glBindTexture( GL_TEXTURE_2D, rightEyeDesc.m_nResolveTextureId );
        else break;
        int cur_tex_w,cur_tex_h;
        glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_WIDTH , &cur_tex_w );
        glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_HEIGHT , &cur_tex_h );
        glTexSubImage2D( GL_TEXTURE_2D, 0, cur_tex_w/2 - hmd_panel_img[i].cols/2,
                                           cur_tex_h/2 - hmd_panel_img[i].rows/2,
                                           hmd_panel_img[i].cols,
                                           hmd_panel_img[i].rows,
                                           GL_RGB, GL_UNSIGNED_BYTE, hmd_panel_img[i].data );
        //std::cout << "img: " << hmd_panel_img[i].cols << " x " << hmd_panel_img[i].rows << std::endl;
        //std::cout << "tex: " << cur_tex_w << " x " << cur_tex_h << std::endl;
//        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture( GL_TEXTURE_2D, 0 );
      }
    }
};
//#endif
#else // USE_IMAGE
// import from opengl sample
std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL )
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
  if( unRequiredBufferLen == 0 )
    return "";

  char *pchBuffer = new char[ unRequiredBufferLen ];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
  std::string sResult = pchBuffer;
  delete [] pchBuffer;
  return sResult;
}
#endif

class VIVEnode
{
  public:
    VIVEnode(int rate);
    ~VIVEnode();
    bool Init();
    void Run();
    void Shutdown();
    void setOriginCB(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
                           std::shared_ptr<std_srvs::srv::Empty::Response> );
    void set_feedback(const sensor_msgs::msg::JoyFeedback &msg);
    //ros::NodeHandle nh_;
    rclcpp::Node::SharedPtr ros2node;
    VRInterface vr_;

#ifdef USE_IMAGE
    void imagesVerticalCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void infoCb(const sensor_msgs::msg::CameraInfo &msg);
    void compressCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg);

    CMainApplicationMod *pMainApplication;
    //image_transport::Subscriber sub_L,sub_R;
    //ros::Subscriber sub_i_L,sub_i_R;
    //rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_i_L,sub_i_R;
    image_transport::Subscriber sub_V;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_i_V;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compress;
#endif

  private:
    rclcpp::Rate loop_rate_;
    rclcpp::Clock ros_clock;
    std::vector<double> world_offset_;
    double world_yaw_;
    tf2_ros::TransformBroadcaster *tf_broadcaster_;
    //tf2_ros::TransformListener tf_listener_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_origin_server_;
#ifdef PUBLISH_TWIST
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist0_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist2_pub_;
#endif
    rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_sub_;
    //std::map<std::string, ros::Publisher> button_states_pubs_map;
};

VIVEnode::VIVEnode(int rate)
  : loop_rate_(rate)
  , vr_()
  , world_offset_({0, 0, 0})
  , world_yaw_(0)
  , ros_clock(RCL_ROS_TIME)
{
    ros2node = rclcpp::Node::make_shared("vive_node");
    tf_broadcaster_ = new tf2_ros::TransformBroadcaster(ros2node);
    if (ros2node->has_parameter("/vive/world_offset")) {
        world_offset_ = ros2node->get_parameter("/vive/world_offset").as_double_array();
    }
    if (ros2node->has_parameter("/vive/world_yaw")) {
        world_yaw_ = ros2node->get_parameter("/vive/world_yaw").as_double();
    }
    //ROS_INFO(" [VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
    set_origin_server_ = ros2node->create_service<std_srvs::srv::Empty>("/vive/set_origin",
                                                                        std::bind(&VIVEnode::setOriginCB, this, _1, _2));
#ifdef PUBLISH_TWIST
    twist0_pub_ = ros2node->create_publisher<geometry_msgs::msg::TwistStamped>("/vive/twist0", 10);
    twist1_pub_ = ros2node->create_publisher<geometry_msgs::msg::TwistStamped>("/vive/twist1", 10);
    twist2_pub_ = ros2node->create_publisher<geometry_msgs::msg::TwistStamped>("/vive/twist2", 10);
#endif
    feedback_sub_ = ros2node->create_subscription<sensor_msgs::msg::JoyFeedback>("/vive/set_feedback", 10,
                                                                                 std::bind(&VIVEnode::set_feedback, this, _1));
#ifdef USE_IMAGE
  image_transport::ImageTransport it(ros2node);
  //sub_L = it.subscribe("/image_left", 1, &VIVEnode::imageCb_L, this);
  //sub_R = it.subscribe("/image_right", 1, &VIVEnode::imageCb_R, this);
  //sub_V = it.subscribe("scene_camera/images",  1, std::bind(&VIVEnode::imagesVerticalCb, this, _1));
  //sub_V = it.subscribe("scene_camera/images/compressed",  1, std::bind(&VIVEnode::imagesVerticalCb, this, _1));
  //sub_V = image_transport::create_subscription(ros2node.get(), "scene_camera/images",
  //std::bind(&VIVEnode::imagesVerticalCb, this, _1), "compressed");
  //sub_i_V = ros2node->create_subscription<sensor_msgs::msg::CameraInfo>("scene_camera/camera_info",  1, std::bind(&VIVEnode::infoCb, this, _1));
  sub_compress = ros2node->create_subscription<sensor_msgs::msg::CompressedImage>("scene_camera/compressed_images",  1,
                                                                                  std::bind(&VIVEnode::compressCb, this, _1));
  pMainApplication = new CMainApplicationMod( 0, NULL );
  if (!pMainApplication->BInit()){
    pMainApplication->Shutdown();
    Shutdown();
  }
  pMainApplication->vr_p = &(vr_);
  pMainApplication->InitTextures();
#endif
  //return;
}

VIVEnode::~VIVEnode()
{
  return;
}

bool VIVEnode::Init()
{
  //  Set logging functions
  vr_.setDebugMsgCallback(handleDebugMessages);
  vr_.setInfoMsgCallback(handleInfoMessages);
  vr_.setErrorMsgCallback(handleErrorMessages);

  if (!vr_.Init())
  {
    return false;
  }

  return true;
}

void VIVEnode::Shutdown()
{
  vr_.Shutdown();
}
void VIVEnode::setOriginCB(const std::shared_ptr<std_srvs::srv::Empty::Request> ,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> )
{
  double tf_matrix[3][4];
  int index = 1, dev_type;
  while (dev_type != 2)
  {
    dev_type = vr_.GetDeviceMatrix(index++, tf_matrix);
  }
  if (dev_type == 0)
  {
    //ROS_WARN(" [VIVE] Coulnd't find controller 1.");
    return;
  }

  //tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
  //                         tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
  //                         tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
  //tf::Vector3 c_z;
  Eigen::Vector3d c_z;
  Eigen::Matrix3d rot_matrix;
  rot_matrix << tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2];
  c_z = rot_matrix * Eigen::Vector3d(0,0,1);
  c_z[1] = 0;
  c_z.normalize();
  double new_yaw = acos(Eigen::Vector3d(0,0,1).dot(c_z)) + M_PI/2;
  if (c_z[0] < 0) new_yaw = -new_yaw;
  world_yaw_ = -new_yaw;

  //tf::Vector3 new_offset;
  //tf::Matrix3x3 new_rot;
  Eigen::Vector3d new_offset;
  Eigen::Matrix3d new_rot(Eigen::AngleAxisd(world_yaw_, Eigen::Vector3d::UnitZ()));
  //new_rot.setRPY(0, 0, world_yaw_);
  new_offset = new_rot * Eigen::Vector3d(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

  world_offset_[0] = new_offset[0];
  world_offset_[1] = new_offset[1];
  world_offset_[2] = new_offset[2];

  //nh_.setParam("/vive/world_offset", world_offset_);
  //nh_.setParam("/vive/world_yaw", world_yaw_);
  //ROS_INFO(" [VIVE] New world offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
  {
      rclcpp::ParameterValue val(world_offset_);
      if (ros2node->has_parameter("/vive/world_offset")) {
          rclcpp::Parameter pp("/vive/world_offset", val);
          ros2node->set_parameter(pp);
      } else {
          ros2node->declare_parameter("/vive/world_offset", val);
      }
  }
  {
      rclcpp::ParameterValue val(world_yaw_);
      if (ros2node->has_parameter("/vive/world_yaw")) {
          rclcpp::Parameter pp("/vive/world_yaw", val);
          ros2node->set_parameter(pp);
      } else {
          ros2node->declare_parameter("/vive/world_yaw", val);
      }
  }
  //return true;
}
void VIVEnode::set_feedback(const sensor_msgs::msg::JoyFeedback &msg) {
  if(msg.type == 1 /* TYPE_RUMBLE */) {
    vr_.TriggerHapticPulse(msg.id, 0, (int)(msg.intensity));
    for(int i=0;i<16;i++)
      vr_.TriggerHapticPulse(i, 0, (int)(msg.intensity));
  }
}

void VIVEnode::Run()
{
  double tf_matrix[3][4];
  int run_hz_count = 0;

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ros2node);
  //while (ros::ok())
  while (rclcpp::ok())
  {
    // do stuff
    vr_.Update();

    int controller_count = 1;
    int tracker_count = 1;
    int lighthouse_count = 1;
    for (int i=0; i<vr::k_unMaxTrackedDeviceCount; i++)
    {
      int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);

      // No device
      if (dev_type == 0) continue;

      Eigen::Translation3d trans(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]);
      Eigen::Matrix3d mat;
      mat << tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
             tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
             tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2];
      Eigen::Isometry3d tf(trans * Eigen::Quaterniond(mat));
      //get device serial number
      std::string cur_sn = GetTrackedDeviceString( vr_.pHMD_, i, vr::Prop_SerialNumber_String );
      std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');

      // It's a HMD
      if (dev_type == 1)
      {
        geometry_msgs::msg::TransformStamped tmp_tf = tf2::eigenToTransform(tf);
        tmp_tf.header.stamp = ros_clock.now();
        tmp_tf.header.frame_id = "world_vive";
        tmp_tf.child_frame_id = "hmd";
        tf_broadcaster_->sendTransform(tmp_tf);
        //tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "hmd"));
      }
      // It's a controller
      if (dev_type == 2)
      {
        geometry_msgs::msg::TransformStamped tmp_tf = tf2::eigenToTransform(tf);
        tmp_tf.header.stamp = ros_clock.now();
        tmp_tf.header.frame_id = "world_vive";
        tmp_tf.child_frame_id = "controller_"+cur_sn;
        tf_broadcaster_->sendTransform(tmp_tf);
        // tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "controller_"+cur_sn));

        vr::VRControllerState_t state;
        vr_.HandleInput(i, state);
        sensor_msgs::msg::Joy joy;
        joy.header.stamp = ros_clock.now();
        joy.header.frame_id = "controller_"+cur_sn;
        joy.buttons.assign(BUTTON_NUM, 0);
        joy.axes.assign(AXES_NUM, 0.0); // x-axis, y-axis
        if((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
          joy.buttons[0] = 1;
        if((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
          joy.buttons[1] = 1;
        if((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
          joy.buttons[2] = 1;
        if((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
          joy.buttons[3] = 1;
        // TrackPad's axis
        joy.axes[0] = state.rAxis[0].x;
        joy.axes[1] = state.rAxis[0].y;
        // Trigger's axis
        joy.axes[2] = state.rAxis[1].x;
//        #include <bitset> // bit debug
//        std::cout << static_cast<std::bitset<64> >(state.ulButtonPressed) << std::endl;
//        std::cout << static_cast<std::bitset<64> >(state.ulButtonTouched) << std::endl;
        //TODO
        //if(button_states_pubs_map.count(cur_sn) == 0){
        //button_states_pubs_map[cur_sn] = nh_.advertise<sensor_msgs::Joy>("/vive/controller_"+cur_sn+"/joy", 10);
        //}
        //button_states_pubs_map[cur_sn].publish(joy);
      }
      // It's a tracker
      if (dev_type == 3)
      {
        geometry_msgs::msg::TransformStamped tmp_tf = tf2::eigenToTransform(tf);
        tmp_tf.header.stamp = ros_clock.now();
        tmp_tf.header.frame_id = "world_vive";
        tmp_tf.child_frame_id = "tracker_"+cur_sn;
        tf_broadcaster_->sendTransform(tmp_tf);
        //tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "tracker_"+cur_sn));
      }
      // It's a lighthouse
      if (dev_type == 4)
      {
        geometry_msgs::msg::TransformStamped tmp_tf = tf2::eigenToTransform(tf);
        tmp_tf.header.stamp = ros_clock.now();
        tmp_tf.header.frame_id = "world_vive";
        tmp_tf.child_frame_id = "lighthouse_"+cur_sn;
        tf_broadcaster_->sendTransform(tmp_tf);
        //tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "lighthouse_"+cur_sn));
      }

    }
    // Publish corrective transform
    //tf::Transform tf_world;
    //tf_world.setOrigin(tf::Vector3(world_offset_[0], world_offset_[1], world_offset_[2]));
    //tf::Quaternion quat_world;
    //quat_world.setRPY(M_PI/2, 0, world_yaw_);
    //tf_world.setRotation(quat_world);
    {
    Eigen::Translation3d trans(world_offset_[0], world_offset_[1], world_offset_[2]);
    Eigen::Quaterniond quat_world(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(world_yaw_, Eigen::Vector3d::UnitZ()));
    //quat_world.setRPY(M_PI/2, 0, world_yaw_);
    Eigen::Isometry3d tf_world (trans * quat_world);
    geometry_msgs::msg::TransformStamped tmp_tf = tf2::eigenToTransform(tf_world);
    tmp_tf.header.stamp = ros_clock.now();
    tmp_tf.header.frame_id = "world";
    tmp_tf.child_frame_id = "world_vive";
    tf_broadcaster_->sendTransform(tmp_tf);
    //tf_broadcaster_.sendTransform(tf::StampedTransform(tf_world, ros::Time::now(), "world", "world_vive"));
    }
#ifdef PUBLISH_TWIST
    // Publish twist messages for controller1 and controller2
    double lin_vel[3], ang_vel[3];
    if (vr_.GetDeviceVel(0, lin_vel, ang_vel))
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = lin_vel[0];
        twist_msg.linear.y = lin_vel[1];
        twist_msg.linear.z = lin_vel[2];
        twist_msg.angular.x = ang_vel[0];
        twist_msg.angular.y = ang_vel[1];
        twist_msg.angular.z = ang_vel[2];

        geometry_msgs::msg::TwistStamped twist_msg_stamped;
        twist_msg_stamped.header.stamp = ros_clock.now();
        twist_msg_stamped.header.frame_id = "world_vive";
        twist_msg_stamped.twist = twist_msg;

        twist0_pub_->publish(twist_msg_stamped);
        // std::cout<<"HMD:";
        // std::cout<<twist_msg_stamped;
    }
    if (vr_.GetDeviceVel(1, lin_vel, ang_vel))
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = lin_vel[0];
        twist_msg.linear.y = lin_vel[1];
        twist_msg.linear.z = lin_vel[2];
        twist_msg.angular.x = ang_vel[0];
        twist_msg.angular.y = ang_vel[1];
        twist_msg.angular.z = ang_vel[2];

        geometry_msgs::msg::TwistStamped twist_msg_stamped;
        twist_msg_stamped.header.stamp = ros_clock.now();
        twist_msg_stamped.header.frame_id = "world_vive";
        twist_msg_stamped.twist = twist_msg;

        twist1_pub_->publish(twist_msg_stamped);
        // std::cout<<"Controller 1:";
        // std::cout<<twist_msg_stamped;
    }
    if (vr_.GetDeviceVel(2, lin_vel, ang_vel))
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = lin_vel[0];
        twist_msg.linear.y = lin_vel[1];
        twist_msg.linear.z = lin_vel[2];
        twist_msg.angular.x = ang_vel[0];
        twist_msg.angular.y = ang_vel[1];
        twist_msg.angular.z = ang_vel[2];

        geometry_msgs::msg::TwistStamped twist_msg_stamped;
        twist_msg_stamped.header.stamp = ros_clock.now();
        twist_msg_stamped.header.frame_id = "world_vive";
        twist_msg_stamped.twist = twist_msg;

        twist2_pub_->publish(twist_msg_stamped);
        // std::cout<<"Controller 2:";
        // std::cout<<twist_msg_stamped;
    }
#endif

#ifdef USE_IMAGE
    pMainApplication->HandleInput();
    //
    //std::cout << "Rf;" << std::endl;
    pMainApplication->RenderFrame();
#endif

    // ROS_INFO_THROTTLE(1.0,"Run() @ %d [fps]", [](int& cin){int ans = cin; cin=0; return ans;}(run_hz_count));
    run_hz_count++;
    exec.spin_all(std::chrono::nanoseconds(10000000000));// timeout 10 sec
    loop_rate_.sleep();
  }
}

#ifdef USE_IMAGE
void VIVEnode::imagesVerticalCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  if(msg->width > 0 && msg->height > 0 ){
    try {
      cv::Mat img_src = cv_bridge::toCvCopy(msg,"rgb8")->image;
      int width =  img_src.cols;
      int height = img_src.rows/2;
      cv::Rect l_roi(0,      0, width, height);
      cv::Rect r_roi(0, height, width, height);
      pMainApplication->ros_img[L] = img_src(l_roi);
      pMainApplication->ros_img[R] = img_src(r_roi);
      // std::cout << width << " x " << height << std::endl;
    } catch (cv_bridge::Exception& e) {
        //ROS_ERROR_THROTTLE(1, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
  }else{
      //ROS_WARN_THROTTLE(3, "Invalid image_right size (%dx%d) use default", msg->width, msg->height);
  }
}
void VIVEnode::infoCb(const sensor_msgs::msg::CameraInfo &msg)
{
  if(msg.k[0] > 0.0 && msg.k[4] > 0.0 ){
    pMainApplication->cam_f[L][0] = pMainApplication->cam_f[R][0] = msg.k[0];
    pMainApplication->cam_f[L][1] = pMainApplication->cam_f[R][1] = msg.k[4];
  }else{
      //ROS_WARN_THROTTLE(3, "Invalid camera_info_left fov (%fx%f) use default", msg.K[0], msg.K[4]);
  }
}
void VIVEnode::compressCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->header = msg->header;
    cv_ptr->image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv_ptr->encoding = msg->format; ////
    try {
      cv::Mat img_src = cv_ptr->image;
      int width =  img_src.cols;
      int height = img_src.rows/2;
      cv::Rect l_roi(0,      0, width, height);
      cv::Rect r_roi(0, height, width, height);
      pMainApplication->ros_img[L] = img_src(l_roi);
      pMainApplication->ros_img[R] = img_src(r_roi);
      // std::cout << width << " x " << height << std::endl;
    } catch (cv_bridge::Exception& e) {
        //ROS_ERROR_THROTTLE(1, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }

}
#endif

#ifdef _WIN32
//// ?? ////
#undef main
#endif
// Main
int main(int argc, char** argv)
{
  signal(SIGINT, mySigintHandler);
  rclcpp::init(argc, argv);

#ifdef USE_IMAGE
  VIVEnode nodeApp(90); // VIVE display max fps
#else
  VIVEnode nodeApp(30);
#endif
  if (!nodeApp.Init()){
    nodeApp.Shutdown();
    return 1;
  }

  nodeApp.Run();
  nodeApp.Shutdown();

  return 0;
}

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_ros/transform_broadcaster.h>
#include "sv/dsol/extra.h"
#include "sv/dsol/node_util.h"
#include "sv/ros1/msg_conv.h"
#include "sv/util/dataset.h"
#include "sv/util/logging.h"
#include "sv/util/ocv.h"
//#include <cv.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

namespace sv::dsol {

using SE3d = Sophus::SE3d;
namespace gm = geometry_msgs;
namespace sm = sensor_msgs;
namespace vm = visualization_msgs;

struct NodeData {
  explicit NodeData(const ros::NodeHandle& pnh);

  void InitOdom();
  void InitRosIO();

  void PublishOdom(const std_msgs::Header& header, const Sophus::SE3d& Twc);
  void PublishCloud(const std_msgs::Header& header) const;
  void SendTransform(const gm::PoseStamped& pose_msg,
                     const std::string& child_frame);
  void ImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,
                     const sensor_msgs::ImageConstPtr& msgRight);
  void DoubleImageCallback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2);
  void Run(cv_bridge::CvImageConstPtr cv_ptrLeft,
           cv_bridge::CvImageConstPtr cv_ptrRight, const ros::Time timestamp);

  double data_max_depth_{0};
  double cloud_max_depth_{100};

//   Dataset dataset_;
  MotionModel motion_;
  TumFormatWriter writer_;
  DirectOdometry odom_;

  KeyControl ctrl_;
  std::string frame_{"fixed"};
  tf2_ros::TransformBroadcaster tfbr_;
  cv::Mat intrin;

  ros::NodeHandle pnh_;
  ros::Publisher clock_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher align_marker_pub_;
  PosePathPublisher kf_pub_;
  PosePathPublisher odom_pub_;

  ros::Publisher points_pub_;

  ros::Time current_frame_time_;


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  //typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Subscriber<sensor_msgs::Image> *left_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *right_sub_;
  message_filters::Synchronizer<sync_pol> *sync_;

  double prev_time {-1};
  int flag {0};
  int cnt {0};
  int buff_count {1};


  bool init_tf{false};
  // The default SE(3) matrix is Identity for Sophus objects
  SE3d dT_pred;
  SE3d T_c0_c_gt;

};

NodeData::NodeData(const ros::NodeHandle& pnh) : pnh_{pnh} {
  frame_ = pnh_.param<std::string>("fix_frame", "fixed");
  ROS_INFO_STREAM("fixed frame: " << frame_);

  InitRosIO();
  InitOdom();
  // Wait after key_control
  const int wait_ms = pnh_.param<int>("wait_ms", 0);
  ROS_INFO_STREAM("wait_ms: " << wait_ms);
  ctrl_ = KeyControl(wait_ms);

  const auto save = pnh_.param<std::string>("save", "");
  writer_ = TumFormatWriter(save);
  if (!writer_.IsDummy()) {
    ROS_WARN_STREAM("Writing results to: " << writer_.filename());
  }

  const auto alpha = pnh_.param<double>("motion_alpha", 0.5);
  motion_ = MotionModel(alpha);
  ROS_INFO_STREAM("motion_alpha: " << motion_.alpha());
}


void NodeData::ImageCallback(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight /*, const sensor_msgs::ImageConstPtr& msgDepth*/) {
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  Run(cv_ptrLeft, cv_ptrRight, cv_ptrLeft->header.stamp);
}  

void SingleImageCallback(const sensor_msgs::ImageConstPtr& image){
  ROS_INFO_STREAM("Received an image in left camera");
  return;
}
void NodeData::DoubleImageCallback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2){
  ROS_INFO_STREAM("Received 2 images in both cameras");
  return;
}

void NodeData::InitRosIO() {
  clock_pub_ = pnh_.advertise<rosgraph_msgs::Clock>("/clock", 1);

  kf_pub_ = PosePathPublisher(pnh_, "kf", frame_);
  odom_pub_ = PosePathPublisher(pnh_, "odom", frame_);
  points_pub_ = pnh_.advertise<sm::PointCloud2>("points", 1);
  pose_array_pub_ = pnh_.advertise<gm::PoseArray>("poses", 1);
  align_marker_pub_ = pnh_.advertise<vm::Marker>("align_graph", 1);

  left_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (pnh_,
      pnh_.param<std::string>("cam1_data", "/Image1/data"), pnh_.param<int>("buff_count", 1));
  right_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (pnh_,
      pnh_.param<std::string>("cam2_data", "/Image2/data"), pnh_.param<int>("buff_count", 1));

  // depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (pnh_, "depth/image_rect_raw", 1);
  
  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
  // sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_, *depth_sub);

  //sync_->registerCallback(boost::bind(&NodeData::DoubleImageCallback, this, _1, _2));
  sync_->registerCallback(boost::bind(&NodeData::ImageCallback, this, _1, _2));
   //sync_->registerCallback(boost::bind(&NodeData::ImageCallback, this, _1, _2, _3));

  //pnh_.getParam("freq", freq_);
  pnh_.getParam("data_max_depth", data_max_depth_);
  pnh_.getParam("cloud_max_depth", cloud_max_depth_);

  intrin = (cv::Mat_<double>(1,5) << pnh_.param<double>("fx",0), pnh_.param<double>("fy",
        0),pnh_.param<double>("cx",0),pnh_.param<double>("cy",0),pnh_.param<double>("bs",0));
  ROS_INFO_STREAM("intrinsics: "<<intrin);
}

void NodeData::InitOdom() {
  {
    auto cfg = ReadOdomCfg({pnh_, "odom"});
    pnh_.getParam("tbb", cfg.tbb);
    pnh_.getParam("log", cfg.log);
    pnh_.getParam("vis", cfg.vis);
    odom_.Init(cfg);
  }
  odom_.selector = PixelSelector(ReadSelectCfg({pnh_, "select"}));
  odom_.matcher = StereoMatcher(ReadStereoCfg({pnh_, "stereo"}));
  odom_.aligner = FrameAligner(ReadDirectCfg({pnh_, "align"}));
  odom_.adjuster = BundleAdjuster(ReadDirectCfg({pnh_, "adjust"}));
  odom_.cmap = GetColorMap(pnh_.param<std::string>("cm", "jet"));

  ROS_INFO_STREAM(odom_.Repr());
}

void NodeData::PublishCloud(const std_msgs::Header& header) const {
  if (points_pub_.getNumSubscribers() == 0) return;
  static sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.point_step = 16;
  cloud.fields = MakePointFieldsXYZI();

  ROS_DEBUG_STREAM(odom_.window.MargKf().status().Repr());

  //Publish in camera frame for straightforward mapping later
  cloud.header.frame_id = "camera";
  Keyframe2CloudCameraFrame(odom_.window.MargKf(), cloud, cloud_max_depth_);
  //Keyframe2Cloud(odom_.window.MargKf(), cloud, cloud_max_depth_);

  points_pub_.publish(cloud);
}

void NodeData::SendTransform(const geometry_msgs::PoseStamped& pose_msg,
                             const std::string& child_frame) {
  gm::TransformStamped tf_msg;
  tf_msg.header = pose_msg.header;
  tf_msg.child_frame_id = child_frame;
  Ros2Ros(pose_msg.pose, tf_msg.transform);
  tfbr_.sendTransform(tf_msg);
}

void NodeData::Run(cv_bridge::CvImageConstPtr cv_ptrLeft, cv_bridge::CvImageConstPtr cv_ptrRight, const ros::Time timestamp) {
  double dt;
  double timestamp_sec = timestamp.toSec();

  if(prev_time < 0){
    motion_.Init(T_c0_c_gt);
    prev_time = timestamp_sec;
    dt = 0;
    flag = 1; //flag is set to 1 only for the first iteration
  }
  else{
    dt = timestamp_sec - prev_time;
    prev_time = timestamp_sec;
    dT_pred = motion_.PredictDelta(dt);
    flag = 0;
  }

  auto image_l = cv_ptrLeft->image;
  auto image_r = cv_ptrRight->image;

  // Intrinsic
  if (!odom_.camera.Ok()){
  // These are for the ERL Realsense camera. On second thought, the order is wrong I believe
  //const cv::Mat intrin({1, 5}, {380.4, 312.9, 379.9, 247.2, 0.095});

  // These are for the Realsense camera from the repo
  //const cv::Mat intrin({1, 5}, {393.4910888671875, 393.4910888671875, 318.6263122558594, 240.12942504882812, 0.095150406});

  // After on-board calibration on ERL realsense
  //K: [430.1014404296875, 0.0, 420.8174133300781, 0.0, 430.1014404296875, 241.85072326660156, 0.0, 0.0, 1.0]
  //const cv::Mat intrin({1, 5}, {430.1014404296875, 430.1014404296875, 420.8174133300781, 241.85072326660156, 0.09493});

  //For the gazebo realsense plugin D435 480x640
  //[347.99755859375, 0.0, 320.0, 0.0, 347.99755859375, 240.0, 0.0, 0.0, 1.0]
  //const cv::Mat intrin({1, 5}, {347.99755859375, 347.99755859375, 320.0, 240.0, 0.05});

  ROS_INFO_STREAM("intrinsics: "<<intrin);
  const auto camera = Camera::FromMat({image_l.cols, image_l.rows}, intrin);
  odom_.SetCamera(camera);
  ROS_INFO_STREAM(camera);
  }

  // Odom
  const auto status = odom_.Estimate(image_l, image_r, dT_pred);
  ROS_INFO_STREAM(status.Repr());

  // Motion model correct if tracking is ok and not first frame
  if (status.track.ok && flag == 0) {
    motion_.Correct(status.Twc(), dt);
  } else {
    ROS_WARN_STREAM("Tracking failed (or 1st frame), slow motion model");
    motion_.Scale(0.5);
  }

  // Write to output
  writer_.Write(cnt++, status.Twc());

  // ROS_DEBUG_STREAM("trans gt:   " << T_c0_c_gt.translation().transpose());
  ROS_DEBUG_STREAM("trans odom: " << status.Twc().translation().transpose());
  ROS_DEBUG_STREAM("trans ba:   " << odom_.window.CurrKf().Twc().translation().transpose());
  ROS_DEBUG_STREAM("aff_l: " << odom_.frame.state().affine_l.ab.transpose());
  ROS_DEBUG_STREAM("aff_r: " << odom_.frame.state().affine_r.ab.transpose());

  // publish stuff
  std_msgs::Header header;
  header.frame_id = frame_;
  header.stamp = timestamp;

  PublishOdom(header, status.Twc());

  if (status.map.remove_kf) {
    PublishCloud(header);
  }
  return;
}

void NodeData::PublishOdom(const std_msgs::Header& header,
                           const Sophus::SE3d& Twc) {
  const auto odom_pose_msg = odom_pub_.Publish(header.stamp, Twc);
  SendTransform(odom_pose_msg, "camera");

  const auto poses = odom_.window.GetAllPoses();
  gm::PoseArray pose_array_msg;
  pose_array_msg.header = header;
  pose_array_msg.poses.resize(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    Sophus2Ros(poses.at(i), pose_array_msg.poses.at(i));
  }
  pose_array_pub_.publish(pose_array_msg);
}

} //namespace closure

int main(int argc, char** argv) {
  ros::init(argc, argv, "dsol_data");
  sv::dsol::NodeData node{ros::NodeHandle{"~"}};
  ros::spin();
  return 0;
}

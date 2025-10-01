#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace astra_camera {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class ConvertRgbNodelet : public nodelet::Nodelet
{
  ros::NodeHandlePtr rgb_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_depth_, sub_rgb_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
  typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
  boost::shared_ptr<Synchronizer> sync_;
  boost::shared_ptr<ExactSynchronizer> exact_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_info_;
  image_transport::Publisher pub_rgb_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::ImageConstPtr& rgb_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void ConvertRgbNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  rgb_nh_.reset( new ros::NodeHandle(nh, "rgb") );
  ros::NodeHandle depth_nh(nh, "depth"); //depth_registered
  rgb_it_  .reset( new image_transport::ImageTransport(*rgb_nh_) );
  depth_it_.reset( new image_transport::ImageTransport(depth_nh) );

  // Read parameters
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool use_exact_sync;
  private_nh.param("exact_sync", use_exact_sync, false);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  if (use_exact_sync)
  {
    exact_sync_.reset( new ExactSynchronizer(ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
    exact_sync_->registerCallback(boost::bind(&ConvertRgbNodelet::imageCb, this, _1, _2, _3));
  }
  else
  {
    sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
    sync_->registerCallback(boost::bind(&ConvertRgbNodelet::imageCb, this, _1, _2, _3));
  }
  
  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ConvertRgbNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_rgb_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_rgb_ = rgb_it_->advertise("image_converted", 1, connect_cb, connect_cb);
  pub_info_ = nh.advertise<sensor_msgs::CameraInfo>("rgb/image_converted/camera_info", 1);
}

// Handles (un)subscribing when clients (un)subscribe
void ConvertRgbNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_rgb_.getNumSubscribers() == 0)
  {
    sub_depth_.unsubscribe();
    sub_rgb_  .unsubscribe();
    sub_info_ .unsubscribe();
  }
  else if (!sub_depth_.getSubscriber())
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";

    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints("raw",ros::TransportHints(), private_nh, depth_image_transport_param);
    sub_depth_.subscribe(*depth_it_, "image_rect",       1, depth_hints);

    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
    sub_rgb_  .subscribe(*rgb_it_,   "image_rect_color", 1, hints);
    sub_info_ .subscribe(*rgb_nh_,   "camera_info",      1);
  }
}

void ConvertRgbNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::ImageConstPtr& rgb_msg_in,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
  {
    NODELET_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                           depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
    return;
  }

  sensor_msgs::CameraInfo info_msg_tmp = *info_msg;

  // Check if the input image has to be resized
  sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
  if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
  {
    info_msg_tmp.width = depth_msg->width;
    info_msg_tmp.height = depth_msg->height;
    float ratio = float(depth_msg->width)/float(rgb_msg->width);
    info_msg_tmp.K[0] *= ratio;
    info_msg_tmp.K[2] *= ratio;
    info_msg_tmp.K[4] *= ratio;
    info_msg_tmp.K[5] *= ratio;
    info_msg_tmp.P[0] *= ratio;
    info_msg_tmp.P[2] *= ratio;
    info_msg_tmp.P[5] *= ratio;
    info_msg_tmp.P[6] *= ratio;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_bridge::CvImage cv_rsz;
    cv_rsz.header = cv_ptr->header;
    cv_rsz.encoding = cv_ptr->encoding;
    cv::resize(cv_ptr->image.rowRange(0,depth_msg->height/ratio), cv_rsz.image, cv::Size(depth_msg->width, depth_msg->height));
    if ((rgb_msg->encoding == enc::RGB8) || (rgb_msg->encoding == enc::BGR8) || (rgb_msg->encoding == enc::MONO8))
      rgb_msg = cv_rsz.toImageMsg();
    else
      rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), enc::RGB8)->toImageMsg();
  } else
    rgb_msg = rgb_msg_in;

  // Supported color encodings: RGB8, BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == enc::RGB8)
  {
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }
  if (rgb_msg->encoding == enc::RGBA8)
  {
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 4;
  }
  else if (rgb_msg->encoding == enc::BGR8)
  {
    red_offset   = 2;
    green_offset = 1;
    blue_offset  = 0;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::BGRA8)
  {
    red_offset   = 2;
    green_offset = 1;
    blue_offset  = 0;
    color_step   = 4;
  }
  else if (rgb_msg->encoding == enc::MONO8)
  {
    red_offset   = 0;
    green_offset = 0;
    blue_offset  = 0;
    color_step   = 1;
  }
  else
  {
    try
    {
      rgb_msg = cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      NODELET_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
      return;
    }
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }

  pub_rgb_.publish(rgb_msg);
  pub_info_.publish(info_msg_tmp);
}

} // namespace astra_camera

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(astra_camera::ConvertRgbNodelet, nodelet::Nodelet)
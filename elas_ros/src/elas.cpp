/*
 Copywrite 2012. All rights reserved.
 Cyphy Lab, https://wiki.qut.edu.au/display/cyphy/Robotics,+Vision+and+Sensor+Networking+at+QUT
 Queensland University of Technology
 Brisbane, Australia

 Author: Patrick Ross
 Contact: patrick.ross@connect.qut.edu.au

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <elas_ros/ElasFrameData.h>

#include <libelas/elas.h>

//#define DOWN_SAMPLE

class Elas_Proc
{
public:
  Elas_Proc(const std::string& transport)
  {
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size_, 5);

    // Topics
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";

    image_transport::ImageTransport it(nh);
    left_sub_.subscribe(it, left_topic, 1, transport);
    right_sub_.subscribe(it, right_topic, 1, transport);
    left_info_sub_.subscribe(nh, left_info_topic, 1);
    right_info_sub_.subscribe(nh, right_info_topic, 1);

    ROS_INFO("Subscribing to:\n%s\n%s\n%s\n%s",left_topic.c_str(),right_topic.c_str(),left_info_topic.c_str(),right_info_topic.c_str());

    image_transport::ImageTransport local_it(local_nh);

    disp_pub_.reset(new Publisher(local_it.advertise("image_disparity", 1)));
    depth_pub_.reset(new Publisher(local_it.advertise("depth", 1)));
    pc_pub_.reset(new ros::Publisher(local_nh.advertise<PointCloud>("point_cloud", 1)));
    elas_fd_pub_.reset(new ros::Publisher(local_nh.advertise<elas_ros::ElasFrameData>("frame_data", 1)));

    pub_disparity_ = local_nh.advertise<stereo_msgs::DisparityImage>("disparity", 1);

    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                                  left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
      approximate_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                                      left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
      exact_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
    }

    // Create the elas processing class
    //param.reset(new Elas::parameters(Elas::MIDDLEBURY));
    //param.reset(new Elas::parameters(Elas::ROBOTICS));
    param.reset(new Elas::parameters);

    /* Parameters tunned*/
    param->disp_min              = 0;
    param->disp_max              = 255;
    param->support_threshold     = 0.95;
    param->support_texture       = 10;
    param->candidate_stepsize    = 5;
    param->incon_window_size     = 5;
    param->incon_threshold       = 5;
    param->incon_min_support     = 5;
    param->add_corners           = 0;
    param->grid_size             = 20;
    param->beta                  = 0.02;
    param->gamma                 = 3;
    param->sigma                 = 1;
    param->sradius               = 2;
    param->match_texture         = 1;
    param->lr_threshold          = 2;
    param->speckle_sim_threshold = 1;
    param->speckle_size          = 200;
    param->ipol_gap_width        = 300;
    param->filter_median         = 0;
    param->filter_adaptive_mean  = 1;
    param->postprocess_only_left = 1;
    param->subsampling           = 0;

    //param->match_texture = 1;
    //param->postprocess_only_left = 1;
    //param->ipol_gap_width = 2;
#ifdef DOWN_SAMPLE
    param->subsampling = true;
#endif
    elas_.reset(new Elas(*param));
  }

  typedef image_transport::SubscriberFilter Subscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
  typedef image_transport::Publisher Publisher;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  void publish_point_cloud(const sensor_msgs::ImageConstPtr& l_image_msg, 
                           float* l_disp_data, const std::vector<int32_t>& inliers,
                           int32_t l_width, int32_t l_height,
                           const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                           const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      pcl::PCLHeader l_info_header = pcl_conversions::toPCL(l_info_msg->header);

      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = l_info_header.frame_id;
      point_cloud->header.stamp = l_info_header.stamp;
      point_cloud->width = 1;
      point_cloud->height = inliers.size();
      point_cloud->points.resize(inliers.size());

      elas_ros::ElasFrameData data;
      data.header.frame_id = l_info_msg->header.frame_id;
      data.header.stamp = l_info_msg->header.stamp;
      data.width = l_width;
      data.height = l_height;
      data.disparity.resize(l_width * l_height);
      data.r.resize(l_width * l_height);
      data.g.resize(l_width * l_height);
      data.b.resize(l_width * l_height);
      data.x.resize(l_width * l_height);
      data.y.resize(l_width * l_height);
      data.z.resize(l_width * l_height);
      data.left = *l_info_msg;
      data.right = *r_info_msg;

      // Copy into the data
      for (int32_t u=0; u<l_width; u++)
      {
        for (int32_t v=0; v<l_height; v++)
        {
          int index = v*l_width + u;
          data.disparity[index] = l_disp_data[index];
#ifdef DOWN_SAMPLE
          cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v*2,u*2);
#else
          cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v,u);
#endif
          data.r[index] = col[0];
          data.g[index] = col[1];
          data.b[index] = col[2];
        }
      }

      for (size_t i=0; i<inliers.size(); i++)
      {
        cv::Point2d left_uv;
        int32_t index = inliers[i];
#ifdef DOWN_SAMPLE
        left_uv.x = (index % l_width) * 2;
        left_uv.y = (index / l_width) * 2;
#else
        left_uv.x = index % l_width;
        left_uv.y = index / l_width;
#endif
        cv::Point3d point;
        model.projectDisparityTo3d(left_uv, l_disp_data[index], point);
        point_cloud->points[i].x = point.x;
        point_cloud->points[i].y = point.y;
        point_cloud->points[i].z = point.z;
        point_cloud->points[i].r = data.r[index];
        point_cloud->points[i].g = data.g[index];
        point_cloud->points[i].b = data.b[index];

        data.x[index] = point.x;
        data.y[index] = point.y;
        data.z[index] = point.z;
      }

      pc_pub_->publish(point_cloud);
      elas_fd_pub_->publish(data);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void process(const sensor_msgs::ImageConstPtr& l_image_msg, 
               const sensor_msgs::ImageConstPtr& r_image_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
               const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {

    ROS_DEBUG("Received images and camera info.");

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);

    // Allocate new disparity image message
    stereo_msgs::DisparityImagePtr disp_msg =
      boost::make_shared<stereo_msgs::DisparityImage>();
    disp_msg->header         = l_info_msg->header;
    disp_msg->image.header   = l_info_msg->header;
    disp_msg->image.height   = l_image_msg->height;
    disp_msg->image.width    = l_image_msg->width;
    disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    disp_msg->image.step     = disp_msg->image.width * sizeof(float);
    disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
    disp_msg->min_disparity = param->disp_min;
    disp_msg->max_disparity = param->disp_max;

    // Stereo parameters
    float f = model_.right().fx();
    float T = model_.baseline();
    float depth_fact = T*f*1000.0f;
    uint16_t bad_point = std::numeric_limits<uint16_t>::max();

    // Have a synchronised pair of images, now to process using elas
    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int32_t l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      l_image_data = const_cast<uint8_t*>(&(l_image_msg->data[0]));
      l_step = l_image_msg->step;
    }
    else
    {
      l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
      l_image_data = l_cv_ptr->image.data;
      l_step = l_cv_ptr->image.step[0];
    }
    if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      r_image_data = const_cast<uint8_t*>(&(r_image_msg->data[0]));
      r_step = r_image_msg->step;
    }
    else
    {
      r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
      r_image_data = r_cv_ptr->image.data;
      r_step = r_cv_ptr->image.step[0];
    }

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

#ifdef DOWN_SAMPLE
    int32_t width = l_image_msg->width/2;
    int32_t height = l_image_msg->height/2;
#else
    int32_t width = l_image_msg->width;
    int32_t height = l_image_msg->height;
#endif

    // Allocate
    const int32_t dims[3] = {l_image_msg->width,l_image_msg->height,l_step};
    //float* l_disp_data = new float[width*height*sizeof(float)];
    float* l_disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);
    float* r_disp_data = new float[width*height*sizeof(float)];

    // Process
    elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);

    // Find the max for scaling the image colour
    float disp_max = 0;
    for (int32_t i=0; i<width*height; i++)
    {
      if (l_disp_data[i]>disp_max) disp_max = l_disp_data[i];
      if (r_disp_data[i]>disp_max) disp_max = r_disp_data[i];
    }

    cv_bridge::CvImage out_depth_msg;
    out_depth_msg.header = l_image_msg->header;
    out_depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
    out_depth_msg.image = cv::Mat(height, width, CV_16UC1);
    uint16_t * out_depth_msg_image_data = reinterpret_cast<uint16_t*>(&out_depth_msg.image.data[0]);

    cv_bridge::CvImage out_msg;
    out_msg.header = l_image_msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = cv::Mat(height, width, CV_8UC1);
    std::vector<int32_t> inliers;
    for (int32_t i=0; i<width*height; i++)
    {
      out_msg.image.data[i] = (uint8_t)std::max(255.0*l_disp_data[i]/disp_max,0.0);
      //disp_msg->image.data[i] = l_disp_data[i];
      //disp_msg->image.data[i] = out_msg.image.data[i]

      float disp =  l_disp_data[i];
      // In milimeters
      //out_depth_msg_image_data[i] = disp;
      out_depth_msg_image_data[i] = disp <= 0.0f ? bad_point : (uint16_t)(depth_fact/disp);

      if (l_disp_data[i] > 0) inliers.push_back(i);
    }

    // Publish
    disp_pub_->publish(out_msg.toImageMsg());
    depth_pub_->publish(out_depth_msg.toImageMsg());
    publish_point_cloud(l_image_msg, l_disp_data, inliers, width, height, l_info_msg, r_info_msg);

    pub_disparity_.publish(disp_msg);

    // Cleanup data
    //delete l_disp_data;
    delete r_disp_data;
  }

private:

  ros::NodeHandle nh;
  Subscriber left_sub_, right_sub_;
  InfoSubscriber left_info_sub_, right_info_sub_;
  boost::shared_ptr<Publisher> disp_pub_;
  boost::shared_ptr<Publisher> depth_pub_;
  boost::shared_ptr<ros::Publisher> pc_pub_;
  boost::shared_ptr<ros::Publisher> elas_fd_pub_;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  boost::shared_ptr<Elas> elas_;
  int queue_size_;

  image_geometry::StereoCameraModel model_;
  ros::Publisher pub_disparity_;
  boost::scoped_ptr<Elas::parameters> param;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elas_ros");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  Elas_Proc processor(transport);

  ros::spin();
  return 0;
}

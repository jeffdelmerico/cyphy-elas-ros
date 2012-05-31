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

#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <elas_ros/ElasFrameData.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>

class PointCloud_Reconstructor
{
public:

  PointCloud_Reconstructor(void)
  {
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size_, 5);

    // Setup the point cloud
    point_cloud.width = 1;
    point_cloud.height = 0;
    base_frame = nh.resolveName("base_frame_id");
    pose_frame = nh.resolveName("pose_frame_id");
    point_cloud.header.frame_id = base_frame;
    first = false;

    // Topics
    std::string elas_frame_data_topic = nh.resolveName("frame_data");
    std::string pose_topic = nh.resolveName("pose");

    // Subscribe
    elas_sub_.subscribe(nh, elas_frame_data_topic, 1);
    pose_sub_.subscribe(nh, pose_topic, 1);
    tf_.reset(new tf::TransformListener(nh));
 
    // Publish
    pc_pub_.reset(new ros::Publisher(local_nh.advertise<PointCloud>("point_cloud", 1)));

    // Set up sync
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_), elas_sub_, pose_sub_));
      approximate_sync_->registerCallback(boost::bind(&PointCloud_Reconstructor::process, this, _1, _2));
    }
    else
    {
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), elas_sub_, pose_sub_));
      exact_sync_->registerCallback(boost::bind(&PointCloud_Reconstructor::process, this, _1, _2));
    }
  }

  void add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
  {
    point_cloud.height++;
    pcl::PointXYZRGB n_pnt;
    n_pnt.x = x;
    n_pnt.y = y;
    n_pnt.z = z;
    n_pnt.r = r;
    n_pnt.g = g;
    n_pnt.b = b;
    point_cloud.points.push_back(n_pnt);
  }

  void process(const elas_ros::ElasFrameDataConstPtr& current_frame_data_, const geometry_msgs::PoseStampedConstPtr& current_pose)
  {
    // Copy everything that's important
    elas_ros::ElasFrameData current_frame_data(*current_frame_data_);

    // Get the transform from the pose frame to the camera frame
    std::string camera_frame = current_frame_data_->header.frame_id;
    tf::StampedTransform trans;
    tf_->waitForTransform(pose_frame, camera_frame, ros::Time::now(), ros::Duration(1.0));
    tf_->lookupTransform(pose_frame, camera_frame, ros::Time(0), trans);

    // Get the transform...
    tf::Pose current_inverse;
    tf::poseMsgToTF(current_pose->pose, current_inverse);
    current_inverse *= trans;
    tf::Pose current_transform = current_inverse.inverse();
    float fc00 = current_transform.getBasis()[0][0]; float fc01 = current_transform.getBasis()[0][1]; 
    float fc02 = current_transform.getBasis()[0][2]; float fc03 = current_transform.getOrigin()[0];
    float fc10 = current_transform.getBasis()[1][0]; float fc11 = current_transform.getBasis()[1][1]; 
    float fc12 = current_transform.getBasis()[1][2]; float fc13 = current_transform.getOrigin()[1];
    float fc20 = current_transform.getBasis()[2][0]; float fc21 = current_transform.getBasis()[2][1]; 
    float fc22 = current_transform.getBasis()[2][2]; float fc23 = current_transform.getOrigin()[2];

    // Apply the transform...
    float rc00 = current_inverse.getBasis()[0][0]; float rc01 = current_inverse.getBasis()[0][1];
    float rc02 = current_inverse.getBasis()[0][2]; float rc03 = current_inverse.getOrigin()[0];
    float rc10 = current_inverse.getBasis()[1][0]; float rc11 = current_inverse.getBasis()[1][1];
    float rc12 = current_inverse.getBasis()[1][2]; float rc13 = current_inverse.getOrigin()[1];
    float rc20 = current_inverse.getBasis()[2][0]; float rc21 = current_inverse.getBasis()[2][1];
    float rc22 = current_inverse.getBasis()[2][2]; float rc23 = current_inverse.getOrigin()[2];

    int32_t current_width = current_frame_data.width;
    int32_t current_height = current_frame_data.height;

    for (int32_t i=0; i < current_width*current_height; i++)
    {
      float x = current_frame_data.x[i]; float y = current_frame_data.y[i]; float z = current_frame_data.z[i];
      float X = rc00*x + rc01*y + rc02*z + rc03;
      float Y = rc10*x + rc11*y + rc12*z + rc13;
      float Z = rc20*x + rc21*y + rc22*z + rc23;
      current_frame_data.x[i] = X;
      current_frame_data.y[i] = Y;
      current_frame_data.z[i] = Z;
    }

    // Get camera data
    image_geometry::PinholeCameraModel l_camera;
    l_camera.fromCameraInfo(current_frame_data_->left);

    if (!first)
    {
      int32_t last_width = last_frame_data.width;
      int32_t last_height = last_frame_data.height;

      // There was a last frame :o, now we get to process
      for (int32_t u=0; u<last_width; u++)
      {
        for (int32_t v=0; v<last_height; v++)
        {
          int32_t last_addr = v*last_width + u;
          float last_d = last_frame_data.disparity[last_addr];

          // Check disparity
          if (last_d > 0.0)
          {
            float last_x = last_frame_data.x[last_addr];
            float last_y = last_frame_data.y[last_addr];
            float last_z = last_frame_data.z[last_addr];

            // Convert into the current reference frame
            float current_x = fc00*last_x + fc01*last_y + fc02*last_z + fc03;
            float current_y = fc10*last_x + fc11*last_y + fc12*last_z + fc13;
            float current_z = fc20*last_x + fc21*last_y + fc22*last_z + fc23;

            // Check in range on current image
            if (current_z>0.5 && current_z<20.0)
            {
              // Compute image coords in current reference frame
              cv::Point3d pnt; pnt.x = current_x; pnt.y = current_y; pnt.z = current_z;
              cv::Point2d current_image_coords = l_camera.project3dToPixel(pnt);

              // Check whether the coords are in the image
              if (current_image_coords.x >= 0 && current_image_coords.x < current_width &&
                  current_image_coords.y >= 0 && current_image_coords.y < current_height)
              {
                int32_t current_addr = ((int32_t)current_image_coords.y)*current_width + ((int32_t)current_image_coords.x);
                float current_d = current_frame_data.disparity[current_addr];

                bool added = false;

                // Check disparity
                if (current_d > 0.0)
                {
                  // Check whether they are close enough to fuse
                  if (fabs(last_x-current_frame_data.x[current_addr]) + 
                      fabs(last_y-current_frame_data.y[current_addr]) +
                      fabs(last_z-current_frame_data.z[current_addr]) < 0.2)
                  {
                    current_frame_data.x[current_addr] = (current_frame_data.x[current_addr]+last_x)/2.0;
                    current_frame_data.y[current_addr] = (current_frame_data.y[current_addr]+last_y)/2.0;
                    current_frame_data.z[current_addr] = (current_frame_data.z[current_addr]+last_z)/2.0;
                    current_frame_data.r[current_addr] = (current_frame_data.r[current_addr] + 
                                                                   last_frame_data.r[last_addr])/2.0;
                    current_frame_data.g[current_addr] = (current_frame_data.g[current_addr] + 
                                                                   last_frame_data.g[last_addr])/2.0;
                    current_frame_data.b[current_addr] = (current_frame_data.b[current_addr] + 
                                                                   last_frame_data.b[last_addr])/2.0;
                    added = true;
                  }
                }
                else
                {
                  // Copy into the current frame
                  current_frame_data.x[current_addr] = last_x;
                  current_frame_data.y[current_addr] = last_y;
                  current_frame_data.z[current_addr] = last_z;
                  current_frame_data.r[current_addr] = last_frame_data.r[last_addr];
                  current_frame_data.g[current_addr] = last_frame_data.g[last_addr];
                  current_frame_data.b[current_addr] = last_frame_data.b[last_addr];

                  added = true;
                }

                if (!added) add_point(last_x, last_y, last_z, 
                                      last_frame_data.r[last_addr], last_frame_data.g[last_addr], last_frame_data.b[last_addr]);
              }
              else add_point(last_x, last_y, last_z, 
                             last_frame_data.r[last_addr], last_frame_data.g[last_addr], last_frame_data.b[last_addr]);
            }
          }
        }
      }
    }
    else first = false;

    // Update the point cloud's header
    point_cloud.header.stamp = current_frame_data_->header.stamp;

    // Publish the cloud...
    pc_pub_->publish(point_cloud);

    // Lastly, save this data
    last_frame_data = current_frame_data;
  }

  typedef message_filters::Subscriber<elas_ros::ElasFrameData> ElasSubscriber;
  typedef message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSubscriber;
  typedef message_filters::sync_policies::ExactTime<elas_ros::ElasFrameData, geometry_msgs::PoseStamped> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<elas_ros::ElasFrameData, geometry_msgs::PoseStamped> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

private:
  ros::NodeHandle nh;
  ElasSubscriber elas_sub_;
  PoseSubscriber pose_sub_;
  boost::shared_ptr<ros::Publisher> pc_pub_;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  boost::shared_ptr<tf::TransformListener> tf_;
  int queue_size_;

  bool first;
  elas_ros::ElasFrameData last_frame_data;
  PointCloud point_cloud;
  std::string base_frame, pose_frame;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc_construction");
  PointCloud_Reconstructor pcr;

  ros::spin();
  return 0;
}

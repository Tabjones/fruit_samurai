/*
 * Software License Agreement (BSD License)
 *
 *   Copyright (c) 2016, Federico Spinelli (fspinelli@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <fruit_samurai/fruit_samurai.h>

namespace fruit_samurai
{
    FruitSamurai::FruitSamurai(const std::string name_space)
    {
        nh_ = boost::make_shared<ros::NodeHandle>(name_space);
        nh_->param<double>("cluster_tolerance", clus_tol_, 0.005);
        nh_->param<bool>("invert_z_projection", invert_, false);
        nh_->param<int>("cluster_min_size", min_size_, 1000);
        nh_->param<int>("cluster_max_size", max_size_, 10000);
        nh_->param<std::string>("input_topic", topic_, "/pacman_vision/processed_scene");
        nh_->param<std::string>("reference_frame", frame_, "/qb_delta_base");
        sub_ = nh_->subscribe(nh_->resolveName(topic_), 1, &FruitSamurai::cbCloud, this);
        sub_req = nh_->subscribe("ask_for_fruit", 1, &FruitSamurai::subCallback, this);
        srv_slice_ = nh_->advertiseService("Slicer", &FruitSamurai::cbSlice, this);
        srv_calib_ = nh_->advertiseService("calibrate", &FruitSamurai::cbCalib, this);
        pub_res = nh_->advertise<std_msgs::Float64MultiArray>("sliced_fruit", 1);
      	delta_trans_.setOrigin(tf::Vector3(-0.017, -0.385, 0.216));
      	delta_trans_.setRotation(tf::Quaternion(-0.188, -0.010, -0.008, 0.982));
    }

    void FruitSamurai::subCallback(const std_msgs::Float64::ConstPtr &msg)
    {

      if (msg->data == 0.0)
      {
        return;
      }

      if (!cloud_){
          ROS_ERROR("[FruitSamurai::%s]\tNo cloud available, check subscriber!", __func__);
          return;
      }
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
      pcl::IndicesClustersPtr clusters = boost::make_shared<pcl::IndicesClusters>();
      ece.setInputCloud(cloud_);
      ece.setClusterTolerance(clus_tol_);
      ece.setMinClusterSize(min_size_);
      ece.setMaxClusterSize(max_size_);
      ece.extract(*clusters);
      ROS_INFO("[FruitSamurai::%s]\tFound %ld fruits!",__func__, clusters->size());
      transf_.clear();
      names_.clear();
      std_msgs::Float64MultiArray fruit_positions;
      for (const auto &cl: *clusters)
      {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
          pcl::copyPointCloud(*cloud_, cl, *cluster);
          names_.push_back(std::string("Fruit_"+std::to_string(cluster->size())));
          //fill in transforms
          Eigen::Vector4f cent, min, max;
          pcl::compute3DCentroid(*cluster, cent);
          pcl::getMinMax3D(*cluster, min, max);
          if (invert_){
              //move towards negative min z
              while (cent[2] >= min[2])
                  cent[2] -= 0.002;
          }
          else{
              //move towards positve max z
              while (cent[2] <= max[2])
                  cent[2] += 0.002;
          }
          Eigen::Vector3f nX,nY,nZ;
          nZ = - Eigen::Vector3f::UnitZ();
          nX = Eigen::Vector3f::UnitX() - (nZ*(nZ.dot(Eigen::Vector3f::UnitX())));
          nX.normalize();
          nY = nZ.cross(nX);
          nY.normalize();
          Eigen::Matrix3f rot;
          rot<< nX(0), nY(0), nZ(0),
                nX(1), nY(1), nZ(1),
                nX(2), nY(2), nZ(2);
          Eigen::Quaternionf q(rot);
          tf::Transform t(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(cent[0], cent[1], cent[2]));
          transf_.push_back(t);
          fruit_positions.data.push_back(cent[0]);
          fruit_positions.data.push_back(cent[1]);
          fruit_positions.data.push_back(cent[2]);
          fruit_positions.data.push_back((double)cluster->size());

          //TODO fill in the response
        }
      pub_res.publish(fruit_positions);
    }

    void FruitSamurai::spinOnce()
    {
        ros::spinOnce();
        if (!transf_.empty()){
            std::size_t n(0);
            for (const auto &t: transf_)
            {
                brcaster_.sendTransform(tf::StampedTransform(
                            t, ros::Time::now(),
                            frame_,
                            names_[n]));
                ++n;
            }
        }
	brcaster_.sendTransform(tf::StampedTransform(
                            delta_trans_, ros::Time::now(),
			    "/camera_rgb_optical_frame",
			    "/qb_delta_base"));
    }

    void FruitSamurai::cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        sensor_msgs::PointCloud msg_conv, msg1;
        sensor_msgs::PointCloud2 msg2;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, msg1);
        listener_.transformPointCloud(frame_, msg1, msg_conv);
        sensor_msgs::convertPointCloudToPointCloud2(msg_conv, msg2);
        pcl::fromROSMsg (msg2, *cloud_);
        //ROS_INFO("Getting cloud");
    }

    bool FruitSamurai::cbSlice(fruit_samurai::Slice::Request &req, fruit_samurai::Slice::Response &res)
    {
        if (!cloud_){
            ROS_ERROR("[FruitSamurai::%s]\tNo cloud available, check subscriber!", __func__);
            return false;
        }
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
        pcl::IndicesClustersPtr clusters = boost::make_shared<pcl::IndicesClusters>();
        ece.setInputCloud(cloud_);
        ece.setClusterTolerance(clus_tol_);
        ece.setMinClusterSize(min_size_);
        ece.setMaxClusterSize(max_size_);
        ece.extract(*clusters);
        ROS_INFO("[FruitSamurai::%s]\tFound %ld fruits!",__func__, clusters->size());
        transf_.clear();
        names_.clear();
        for (const auto &cl: *clusters)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            pcl::copyPointCloud(*cloud_, cl, *cluster);
            names_.push_back(std::string("Fruit_"+std::to_string(cluster->size())));
            //fill in transforms
            Eigen::Vector4f cent, min, max;
            pcl::compute3DCentroid(*cluster, cent);
            pcl::getMinMax3D(*cluster, min, max);
            if (invert_){
                //move towards negative min z
                while (cent[2] >= min[2])
                    cent[2] -= 0.002;
            }
            else{
                //move towards positve max z
                while (cent[2] <= max[2])
                    cent[2] += 0.002;
            }
            Eigen::Vector3f nX,nY,nZ;
            nZ = - Eigen::Vector3f::UnitZ();
            nX = Eigen::Vector3f::UnitX() - (nZ*(nZ.dot(Eigen::Vector3f::UnitX())));
            nX.normalize();
            nY = nZ.cross(nX);
            nY.normalize();
            Eigen::Matrix3f rot;
            rot<< nX(0), nY(0), nZ(0),
                  nX(1), nY(1), nZ(1),
                  nX(2), nY(2), nZ(2);
            Eigen::Quaternionf q(rot);
            tf::Transform t(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(cent[0], cent[1], cent[2]));
            transf_.push_back(t);
            std_msgs::Float64 x_f, y_f, z_f;
            x_f.data = cent[0];
            y_f.data = cent[1];
            z_f.data = cent[2];
            res.x.push_back(x_f);
            res.y.push_back(y_f);
            res.z.push_back(z_f);
        }
        return true;
    }
bool FruitSamurai::cbCalib(fruit_samurai::calibrate::Request &req, fruit_samurai::calibrate::Response &res)
{
	tf::StampedTransform mark_cam, mark_delta;
	mark_delta.setOrigin(tf::Vector3(0,0,0.5075));
	mark_delta.setRotation(tf::Quaternion(1,0,0,0));
	try
	{
		listener_.waitForTransform("/camera_rgb_optical_frame", "/ar_marker_50", ros::Time(0), ros::Duration(5));
		listener_.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_50", ros::Time(0), mark_cam);
	}
	catch ( tf::TransformException ex)
	{
		return false;
	}
	delta_trans_ = mark_cam*mark_delta.inverse();
return true;
}

} //End namespace

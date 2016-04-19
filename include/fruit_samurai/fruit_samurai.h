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
 * * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef _INCL_FRUIT_SAMURAI_H_
#define _INCL_FRUIT_SAMURAI_H_

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/console.h>
#include <ros/common.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/rate.h>
#include <fruit_samurai/Slice.h>

namespace fruit_samurai
{
    class FruitSamurai
    {
        public:
        FruitSamurai()=delete;
        FruitSamurai(const std::string name_space);

        ///Get NodeHandle
        inline ros::NodeHandle getNodeHandle() const
        {
            return *nh_;
        }
        ///Main spin method
        void spinOnce();

        private:
        ///Callback to get input point cloud
        void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        ///Slice service callback perform fruit slicing, aka segmentation in the box
        bool cbSlice(fruit_samurai::Slice::Request &req, fruit_samurai::Slice::Response &res);
        boost::shared_ptr<ros::NodeHandle> nh_;
        ros::Subscriber sub_;
        ros::ServiceServer srv_slice_;
        tf::TransformBroadcaster fruit_brcaster_;
        tf::TransformListener listener_;

        std::string topic_, frame_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
        std::vector<tf::Transform> transf_;
        std::vector<std::string> names_;
        //params
        double clus_tol_;
        int min_size_, max_size_;
        bool invert_;
    };
}
#endif //_INCL_FRUIT_SAMURAI_H_

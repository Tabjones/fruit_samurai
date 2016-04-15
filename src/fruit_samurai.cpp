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
    FruitSamurai::FruitSamurai(const std::string name_space):
        disabled_(false), topic_("/pacman_vision/processed_scene")
    {
        nh_ = boost::make_shared<ros::NodeHandle>(name_space);
        //TODO services, subscribers
        //TODO rosparams
        nh_->subscribe(nh_->resolveName(topic_), 1, &FruitSamurai::cbCloud, this);
        nh_->advertiseService("slice", &FruitSamurai::cbSlice, this);
    }

    void FruitSamurai::spinOnce() const
    {
        ros::spinOnce();
        if (disabled_)
            return;
        //TODO add tf broadcaster
    }

    void FruitSamurai::cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        //TODO this does not use the disabled features
        cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg (*msg, *cloud_);
    }

    bool FruitSamurai::cbSlice(fruit_samurai::Slice::Request &req, fruit_samurai::Slice::Response &res)
    {
        if (disabled_){
            ROS_ERROR("[FruitSamurai::%s]\tNode is disabled, this service is suspended!", __func__);
            return false;
        }
        //TODO
    }
}

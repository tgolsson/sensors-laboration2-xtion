/*
 * This source file is part of the Sensors and Sensing course at AASS.
 * If you use this material in your courses or research, please include 
 * a reference.
 * 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

//include headers from ROS
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//PCL
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
//EIGEN
#include <Eigen/Eigen>
//OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//OPENCV Window names
#define RGB_WINDOW "RGB Image"
#define DEPTH_WINDOW "Depth Image"

//Your Node Class
class AsusNode {

    private:
    // Our NodeHandle, points to home
    ros::NodeHandle nh_;
    //global node handle
    ros::NodeHandle n_;

    //Subscribers for topics
    ros::Subscriber points_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;

    //topics to subscribe to
    std::string subscribe_topic_point;
    std::string subscribe_topic_depth;
    std::string subscribe_topic_color;

    public:
    AsusNode() {

	nh_ = ros::NodeHandle("~");
	n_ = ros::NodeHandle();
	
	//read in topic names from the parameter server
	nh_.param<std::string>("points_topic",subscribe_topic_point,"/camera/depth_registered/points");
	nh_.param<std::string>("depth_topic",subscribe_topic_depth,"/camera/depth_registered/image_raw");
	nh_.param<std::string>("rgb_topic",subscribe_topic_color,"/camera/rgb/image_raw");

	//subscribe to topics
	points_sub_ = n_.subscribe(subscribe_topic_point, 1, &AsusNode::points2Callback, this);
	depth_sub_ = n_.subscribe(subscribe_topic_depth, 1, &AsusNode::depthCallback, this);
	rgb_sub_ = n_.subscribe(subscribe_topic_color, 1, &AsusNode::rgbCallback, this);

	//create opencv windows
	cv::namedWindow(RGB_WINDOW);
	cv::namedWindow(DEPTH_WINDOW);
    }

    // Callback for pointclouds
    void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
    {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*msg_in, cloud);
	pcl::io::savePCDFileASCII ("pcloud.pcd", cloud);
		
	/* do something pointy"*/
	ROS_INFO_STREAM("Got cloud with "<<cloud.size()<<" points");

    }

    //callback for rgb images
    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
	cv_bridge::CvImageConstPtr bridge;
	try
	{
	    bridge = cv_bridge::toCvCopy(msg, "bgr8");
	    cv::FileStorage fs("rgbbmp.yml", cv::FileStorage::WRITE);
	    fs << "imagergb" << bridge->image;
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("Failed to transform rgb image.");
	    return;

	}
	/* do something colorful"*/
	cv::imshow(RGB_WINDOW, bridge->image);
	cv::waitKey(1);
    }

    //callback for RGB images
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
	cv_bridge::CvImageConstPtr bridge;
	try
	{
	    bridge = cv_bridge::toCvCopy(msg, "32FC1");
	    cv::FileStorage fs("rgbdepth.yml", cv::FileStorage::WRITE);
	    fs << "imagedepth" << bridge->image;
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("Failed to transform depth image.");
	    return;
	}
	/* do something depthy"*/
	cv::imshow(DEPTH_WINDOW, bridge->image);
	cv::waitKey(1);
    }

};

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "asus_node");

    std::cerr<<"creating node\n";
    AsusNode nd;
    std::cerr<<"node done\n";
    ros::spin();

    return 0;
}


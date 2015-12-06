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
#include <opencv2/imgproc/imgproc.hpp>
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>


#include <vector>
#include <algorithm>
//OPENCV Window names
#define RGB_WINDOW "RGB Image"
#define DEPTH_WINDOW "Depth Image"
#define DEPTH_WINDOW_CENTER "Depth Image Center"
#define X_COUNT 640
#define Y_COUNT 480
#define X_SIZE 40
#define Y_SIZE 40

//Your Node Class
class AsusNode
{

private:

    // Constants
    const int CUTOFF_DISTANCE = 10;

    
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
		
    rosbag::Bag bag;
    cv::Mat buffer[10];
    int buffPoint, numElements;


    // Caclulates the mean and variance for input matrix
    void calculateMeanVariance(cv::Mat matrix, float * meanOut, float * varianceOut)
    {
        // Init variables
        std::vector<float> values;
        float sum, tempMean, tempVariance;
        sum = tempMean = tempVariance = 0;

        // Iterate over the matrix and add proper values to the vector
        for (cv::MatConstIterator_<float> it = matrix.begin<float>(); it != matrix.end<float>(); it++)
        {
            if (*it > 0 && *it < CUTOFF_DISTANCE)
            {
                sum += *it;
                values.push_back(*it);
            }
        }

        // If all values are bad and we got nothing back
        // NOTE: Being 0 when not having any values is not correct... But an assert will be very disruptive.
        if (values.size() == 0) {
            tempMean = 0;
            tempVariance = 0;
        }
        else
        {
            // Calculate mean
            tempMean = sum / (float)values.size();

            // Calculate variance
            float varianceSum = 0;
            for (std::vector<float>::iterator it = values.begin(); it != values.end(); it++)            {
                varianceSum = pow(*it - tempMean, 2);
            }
            tempVariance = varianceSum / (float) values.size();
        }

        //Return values
        *meanOut = tempMean;
        *varianceOut = tempVariance;
    }
   
public:
    int stopCounter;

    AsusNode()
    {
	nh_ = ros::NodeHandle("~");
	n_ = ros::NodeHandle();

        // Initialize variables
	bag.open("report.bag", rosbag::bagmode::Write);
	buffPoint = numElements = 0;
	for (int i=0; i<10; i++)
        {
            buffer[i].create(X_SIZE, Y_SIZE, CV_32F);
	}
        stopCounter = 0;

        // read in topic names from the parameter server
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
	cv::namedWindow(DEPTH_WINDOW_CENTER);
    }

    // Callback for pointclouds
    void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (*msg_in, cloud);

        //Write to file
        //        pcl::io::savePCDFileASCII ("pcloud.pcd", cloud);
    }

    //callback for rgb images
    void rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr bridge;
        try
        {
            bridge = cv_bridge::toCvCopy(msg, "bgr8");

            // Write to file
            cv::FileStorage fs("rgbbmp.yml", cv::FileStorage::WRITE);
            //fs << "imagergb" << bridge->image;
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
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg){
        stopCounter = 0;
        cv_bridge::CvImageConstPtr bridge;
        try{
            bridge = cv_bridge::toCvCopy(msg, "32FC1");

            //write to file
            cv::FileStorage fs("rgbdepth.yml", cv::FileStorage::WRITE);
            //fs << "imagedepth" << bridge->image;
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Failed to transform depth image.");
            return;
        }
        /* do something depthy"*/
        cv::imshow(DEPTH_WINDOW, bridge->image);
        cv::waitKey(1);
	

        // cut out the center
        cv::Range dx = cv::Range(X_COUNT/2 - X_SIZE/2, X_COUNT/2 + X_SIZE/2);
        cv::Range dy = cv::Range(Y_COUNT/2 - Y_SIZE/2, Y_COUNT/2 + Y_SIZE/2);
        cv::Mat submatrix = bridge->image(dy, dx);
        
        // Do the gaussian and median before cleanup
        cv::Mat gaussian, median;
        cv::GaussianBlur(submatrix, gaussian, cv::Size(5,5), 0,0);
        cv::medianBlur(submatrix, median, 5);

        // Do the mask and cleanup nans
        cv::Mat mask;
        mask.create(X_SIZE,Y_SIZE, CV_32F);
        for (int x = 0; x < X_SIZE; x++)
        {
            for (int y=0; y < Y_SIZE; y++)
            {
                if (std::isnan(submatrix.at<float>(x,y)))
                {
                    mask.at<float>(x,y) = 0.0f;
                    submatrix.at<float>(x,y) = 10000;
                }
                else
                {
                    mask.at<float>(x,y) = 1.0f;
                }
            }
        }

        // Do the bilateral
        cv::Mat bilateral;
        cv::bilateralFilter(submatrix, bilateral, 5, 5*2, 5/2);
	

        // Circular buffer for the (cleaned) center images
        buffer[buffPoint] = submatrix;
        buffPoint = (buffPoint + 1) % 10;
        if (numElements < 10){
            numElements++;
        }


        // Median and average matrices
        cv::Mat outputM(X_SIZE,Y_SIZE,CV_32F, cv::Scalar(0.0f));
        cv::Mat outputA(X_SIZE,Y_SIZE,CV_32F, cv::Scalar(0.0f));

        // Calculate temporal and median 
        for (int y = 0; y < Y_SIZE; y++)
        { 
            for (int x = 0; x < X_SIZE;  x++)
            {
                // Separate all the good values from bad ones        
                std::vector<float> numbers;
                for (int i = 0; i < numElements; i++)
                {
                    float val = buffer[i].at<float>(x,y);
                    if (val < CUTOFF_DISTANCE && val > 0)
                    {
                        numbers.push_back(val);
                    }
                }

                int length = numbers.size();

                // Sort for median
                std::sort(numbers.begin(), numbers.end());
                if (length > 1)
                {
                    // Median
                    if (numbers.size() % 2 == 0)
                    {
                        outputM.at<float>(x,y) = (numbers[(int)length/2 - 1]+numbers[(int)length/2])/2.0f;
                    }
                    else
                    {
                        outputM.at<float>(x,y) = numbers[(int)length/2];
                    }

                    // Mean
                    float sum = 0;
                    for (int i=0; i < length; i++)
                    {
                        sum += numbers[i];       
                    }
                    outputA.at<float>(x,y) = sum / (float)length;
                }
                else
                {
                    // NOTE: Setting to 0 might not be the best approach, though we know it's an invalid value (because 0 distance makes no sense)
                    outputA.at<float>(x,y) = 0;
                    outputM.at<float>(x,y) = 0;
                }
            }
        }
        
        

        // Mask out all the huge values from before
        cv::multiply(submatrix, mask, submatrix);
        cv::multiply(bilateral, mask, bilateral);
        cv::multiply(gaussian, mask, gaussian);
        cv::multiply(median, mask, median);


        // // Create and display original and filtered cneters
        // cv::Mat gathered(X_SIZE*2+5, Y_SIZE*3+10, CV_32F, cv::Scalar(0.0f));
        // cv::Range r1(0, X_SIZE);
        // cv::Range r2(X_SIZE+5, X_SIZE*2+5);
        // cv::Range r3(X_SIZE*2+10, X_SIZE*3+10);
        // // First row
        // submatrix.copyTo(gathered.colRange(r1).rowRange(r1));
        // gaussian.copyTo(gathered.colRange(r2).rowRange(r1));
        // median.copyTo(gathered.colRange(r3).rowRange(r1));

        // // Second row
        // bilateral.copyTo(gathered.colRange(r1).rowRange(r2));
        // outputA.copyTo(gathered.colRange(r2).rowRange(r2));
        // outputM.copyTo(gathered.colRange(r3).rowRange(r2));

        // // Display
        // cv::imshow(DEPTH_WINDOW_CENTER, gathered);
        // cv::waitKey(1);

        //Create and send messages
        std_msgs::Float32 meanMsg, varianceMsg;
        std_msgs::Float32 meanMsgG, varianceMsgG;
        std_msgs::Float32 meanMsgM, varianceMsgM;
        std_msgs::Float32 meanMsgB, varianceMsgB;
        std_msgs::Float32 meanMsgtM, varianceMsgtM;
        std_msgs::Float32 meanMsgtA, varianceMsgtA;

        calculateMeanVariance(submatrix, &meanMsg.data,&varianceMsg.data);
        calculateMeanVariance(gaussian, &meanMsgG.data,&varianceMsgG.data);        
        calculateMeanVariance(bilateral, &meanMsgB.data,&varianceMsgB.data);
        calculateMeanVariance(median, &meanMsgM.data,&varianceMsgM.data);        
        calculateMeanVariance(outputA, &meanMsgtA.data,&varianceMsgtA.data);        
        calculateMeanVariance(outputM, &meanMsgtM.data,&varianceMsgtM.data);

        bag.write("mean", ros::Time::now(), meanMsg);
        bag.write("meanG", ros::Time::now(), meanMsgG);
        bag.write("meanB", ros::Time::now(), meanMsgB);
        bag.write("meanM", ros::Time::now(), meanMsgM);
        bag.write("meantM", ros::Time::now(), meanMsgtM);
        bag.write("meantA", ros::Time::now(), meanMsgtA);
        
        bag.write("variance", ros::Time::now(), varianceMsg);
        bag.write("varianceG", ros::Time::now(), varianceMsgG);
        bag.write("varianceB", ros::Time::now(), varianceMsgB);
        bag.write("varianceM", ros::Time::now(), varianceMsgM);
        bag.write("variancetM", ros::Time::now(), varianceMsgtM);
        bag.write("variancetA", ros::Time::now(), varianceMsgtA);
    }
    
};

//main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "asus_node");

    std::cerr<<"creating node\n";
    AsusNode nd;
    
    // 30 hz loop
    ros::Rate r(60);

    //Shutdown if it has gone 1 minute without messages received
    while (nd.stopCounter < 30)
    {
        nd.stopCounter++; 
        ros::spinOnce();
        r.sleep();
    }
    std::cerr<<"node done\n";
    return 0;
}


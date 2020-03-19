// (MIT License)
//
// Copyright 2019 David B. Curtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// Subscribes to an image stream of side-by-side stereo where each image
// message consists of a left and right image concatenated to form a single
// double-wide image.  This node splits the incoming image down the middle
// and republishes each half as stereo/left and stereo/right images.
//
// This is a modified version of public domain code posted by PeteBlackerThe3rd
// in response to my question on ROS Answers:
// https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/
//
// -- dbc

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// size of the output image
int outputWidth, outputHeight;

image_transport::Publisher left_rePublisher;
image_transport::Publisher right_rePublisher;

// Image capture callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("input image");
    // get double camera image
    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, "rgb8");
    cv::Mat image = cvImg->image;

        // define the relevant rectangle to crop
    	cv::Rect ROI_left;
    	cv::Rect ROI_right;
        ROI_left.y = 0;
        ROI_left.width = image.cols / 2;
        ROI_left.height = image.rows;
	ROI_right.y = 0;
        ROI_right.width = image.cols / 2;
        ROI_right.height = image.rows;
        ROI_left.x = 0;
        ROI_right.x = image.cols / 2;

        // crop left image 
        cv::Mat croppedImage_left = cv::Mat(image, ROI_left);

        cv::Mat scaledImage_left;
        cv::resize(croppedImage_left,
                   scaledImage_left,
                   cv::Size(outputWidth, outputHeight) );

        cv_bridge::CvImage cvImage_left;
        cvImage_left.image = scaledImage_left;
        cvImage_left.encoding = "bgr8";
   
	//crop right image
	cv::Mat croppedImage_right = cv::Mat(image, ROI_right);

        cv::Mat scaledImage_right;
        cv::resize(croppedImage_right,
                   scaledImage_right,
                   cv::Size(outputWidth, outputHeight) );

        cv_bridge::CvImage cvImage_right;
        cvImage_right.image = scaledImage_right;
        cvImage_right.encoding = "bgr8";        

        //publish left and right images
        left_rePublisher.publish(cvImage_left.toImageMsg());
        right_rePublisher.publish(cvImage_right.toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "half_image_publisher_node");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);

    // load node settings
    std::string inputTopic, outputTopic_l, outputTopic_r;
    //nh.param("image_side", side, std::string("left"));
    nh.param("input_topic", inputTopic, std::string("/rtsp/image_raw"));
    nh.param("output_topic_l", outputTopic_l, std::string("/left/image_raw"));
    nh.param("output_topic_r", outputTopic_r, std::string("/right/image_raw"));
    nh.param("width", outputWidth, 640);
    nh.param("height", outputHeight, 480);

    // register publisher and subscriber
    ros::Subscriber imageSub = nh.subscribe(inputTopic.c_str(), 2, &imageCallback);

    left_rePublisher = it.advertise(outputTopic_l.c_str(), 1);
    right_rePublisher = it.advertise(outputTopic_r.c_str(), 1);
    // run node until cancelled
    ros::spin();
}

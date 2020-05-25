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
// https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright
// -- dbc

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// size of the output image
int outputWidth, outputHeight;

image_transport::Publisher left_rePublisher;
image_transport::Publisher right_rePublisher;

//left and right camera info publisher, and hardcoded info
//to do: make it read from a calibration file
ros::Publisher leftCameraInfoPublisher, rightCameraInfoPublisher;
sensor_msgs::CameraInfo leftCameraInfoMsg, rightCameraInfoMsg;

// hardcode the camera calibration results, to do: optimize it like reading the config file
void config()
{
	leftCameraInfoMsg.height = 480;
	leftCameraInfoMsg.width = 640;
	leftCameraInfoMsg.distortion_model = "plumb_bob";
	leftCameraInfoMsg.D.resize(5);
	float lD[5] = {0.130184,-0.406491,-0.000160,-0.005097,0.000000};
	leftCameraInfoMsg.D.assign(lD, lD+5);
	leftCameraInfoMsg.K = {1225.9684830346469, 0.0, 287.9583557799272, 0.0, 				608.3046087452668, 209.9193382735547, 0.0, 0.0, 				1.0};
	leftCameraInfoMsg.R = {0.9992528217718623, -0.015592515707818128, 					0.03536483613523994, 0.014697570115292833, 					0.9995686461025457, 0.025426426438444085, 					-0.0357460433289753, -0.024887651207476408, 					0.9990509622655382};
	leftCameraInfoMsg.P = {1497.133646596029, 0.0, 280.32798194885254, 0.0, 				0.0, 1497.133646596029, 209.10777473449707, 0.0, 					0.0, 0.0, 1.0, 0.0};
	leftCameraInfoMsg.binning_x = 1;
	leftCameraInfoMsg.binning_y = 1;

	rightCameraInfoMsg.height = 480;
	rightCameraInfoMsg.width = 640;
	rightCameraInfoMsg.distortion_model = "plumb_bob";
        rightCameraInfoMsg.D.resize(5);
	float rD[5] = {0.12336810102604828, -0.5262842154518701, 				-0.0067526064540203755, 0.0016888408837813604, 0.0};
	rightCameraInfoMsg.D.assign(rD, rD+5);
	rightCameraInfoMsg.K = {1222.45392141489, 0.0, 300.3108491157346, 0.0, 					606.8361376824262, 212.8480490740382, 0.0, 0.0, 				1.0};
	rightCameraInfoMsg.R = {0.9987677360311582, -0.015000711432805522, 					0.04730737912529363, 0.016186661970008266, 					0.9995617175356302, -0.02478638359038563, 					-0.04691483174288552, 0.025521588777557457, 					0.9985728050917498};
	rightCameraInfoMsg.P = {1497.133646596029, 0.0, 280.32798194885254, 					-2.4085914533138992, 0.0, 1497.133646596029, 					209.10777473449707, 0.0, 0.0, 0.0, 1.0, 0.0};
	rightCameraInfoMsg.binning_x = 1;
	rightCameraInfoMsg.binning_y = 1;
}

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
        cvImage_left.encoding = "rgb8";
   
	//crop right image
	cv::Mat croppedImage_right = cv::Mat(image, ROI_right);

        cv::Mat scaledImage_right;
        cv::resize(croppedImage_right,
                   scaledImage_right,
                   cv::Size(outputWidth, outputHeight) );

        cv_bridge::CvImage cvImage_right;
        cvImage_right.image = scaledImage_right;
        cvImage_right.encoding = "rgb8";        

        //publish left and right images
        left_rePublisher.publish(cvImage_left.toImageMsg());
        right_rePublisher.publish(cvImage_right.toImageMsg());
	
	leftCameraInfoPublisher.publish(leftCameraInfoMsg);
	rightCameraInfoPublisher.publish(rightCameraInfoMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_splitter_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_left;
    ros::NodeHandle nh_right;
    //init camera info
    config();
    image_transport::ImageTransport it(nh);

    // load node settings
    std::string inputTopic, outputTopic_l, outputTopic_r;
    std::string leftCameraInfo, rightCameraInfo;
    //nh.param("image_side", side, std::string("left"));
    nh.param("input_topic", inputTopic, std::string("/rtsp/image_raw"));
    nh.param("output_topic_l", outputTopic_l, std::string("/stereopi/left/image_raw"));
    nh.param("output_topic_r", outputTopic_r, std::string("/stereopi/right/image_raw"));
    nh.param("width", outputWidth, 640);
    nh.param("height", outputHeight, 480);
    nh.param("left_camera_info_topic", leftCameraInfo,
        std::string("/stereopi/left/camera_info"));
    nh.param("right_camera_info_topic", rightCameraInfo,
        std::string("/stereopi/right/camera_info"));

    // register publisher and subscriber
    ros::Subscriber imageSub = nh.subscribe(inputTopic.c_str(), 2, &imageCallback);

    left_rePublisher = it.advertise(outputTopic_l.c_str(), 1);
    right_rePublisher = it.advertise(outputTopic_r.c_str(), 1);
    leftCameraInfoPublisher = nh_left.advertise<sensor_msgs::CameraInfo>
        (leftCameraInfo.c_str(), 1, true);
    rightCameraInfoPublisher = nh_right.advertise<sensor_msgs::CameraInfo>
        (rightCameraInfo.c_str(), 1, true);
    // run node until cancelled
    ros::spin();
}

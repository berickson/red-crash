#include "ros/ros.h"
#include <ros/console.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include "stereo_pipeline.hpp"
#include <functional>

#include "depthai/depthai.hpp"

#include <cv_bridge/cv_bridge.h>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <image_transport/image_transport.h>
#include <thread>
#include <chrono>


int main(int argc, char** argv)  {

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle node_handle("~");
    ROS_INFO("starting oakd_cpp::StereoNodelet");
    
    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;
    int queue_length = 1;

    if(!node_handle.getParam("camera_name", deviceName))
    {
        ROS_ERROR("Couldn't getParam camera_name");
        throw std::runtime_error("Couldn't getParam camera_name");
    }
    if(!node_handle.getParam("camera_param_uri", camera_param_uri))
    {
        ROS_ERROR("Couldn't getParam camera_param_uri");
        throw std::runtime_error("Couldn't getParam camara_param_uri");
    }

    StereoPipeline pipeline;
    pipeline.initDepthaiDev();
    // dai::Device device(pipeline._p);

    auto queue_left = pipeline._dev->getOutputQueue("left");
    auto queue_right = pipeline._dev->getOutputQueue("right");
    auto queue_depth = pipeline._dev->getOutputQueue("depth");
    auto queue_rectified_left = pipeline._dev->getOutputQueue("rectified_left");
    auto queue_rectified_right = pipeline._dev->getOutputQueue("rectified_right");


    image_transport::ImageTransport image_transport(node_handle);
    auto pub_left = image_transport.advertise("oakd/left", queue_length);
    auto pub_right = image_transport.advertise("oakd/right", queue_length);
    auto pub_depth = image_transport.advertise("oakd/depth", queue_length);
    auto pub_rectified_left = image_transport.advertise("oakd/rectified_left", queue_length);
    auto pub_rectified_right = image_transport.advertise("oakd/rectified_right", queue_length);

    while(ros::ok()) {
        auto left_frame = queue_left->tryGet<dai::ImgFrame>();
        if(left_frame && pub_left.getNumSubscribers()) {
            static int i = 0;
            static dai::rosBridge::ImageConverter image_converter("oakd");
            static sensor_msgs::Image msg;
            std::cout << "left frame " << i << std::endl;
            image_converter.toRosMsg(left_frame, msg);
            pub_left.publish(msg);
            ++i;
        }
        auto rectified_left_frame = queue_rectified_left->tryGet<dai::ImgFrame>();
        if(rectified_left_frame && pub_rectified_left.getNumSubscribers()) {
            static int i = 0;
            static dai::rosBridge::ImageConverter image_converter("oakd");
            static sensor_msgs::Image msg;
            std::cout << "rectified_left frame " << i << std::endl;
            image_converter.toRosMsg(rectified_left_frame, msg);
            pub_rectified_left.publish(msg);
            ++i;
        }


        auto rectified_right_frame = queue_rectified_right->tryGet<dai::ImgFrame>();
        if(rectified_right_frame && pub_rectified_right.getNumSubscribers()) {
            static int i = 0;
            static dai::rosBridge::ImageConverter image_converter("oakd");
            static sensor_msgs::Image msg;
            std::cout << "rectified_right frame " << i << std::endl;
            image_converter.toRosMsg(rectified_right_frame, msg);
            pub_rectified_right.publish(msg);
            ++i;
        }
        auto right_frame = queue_right->tryGet<dai::ImgFrame>();
        if(right_frame && pub_right.getNumSubscribers()) {
            static int i = 0;
            static dai::rosBridge::ImageConverter image_converter("oakd");
            static sensor_msgs::Image msg;
            std::cout << "right frame " << i << std::endl;
            image_converter.toRosMsg(right_frame, msg);
            pub_right.publish(msg);
            ++i;
        }
        auto depth_frame = queue_depth->tryGet<dai::ImgFrame>();
        if(depth_frame && pub_depth.getNumSubscribers()) {
            static int i = 0;
            static dai::rosBridge::ImageConverter image_converter("oakd");
            static sensor_msgs::Image msg;
            std::cout << "depth frame " << i << std::endl;
            image_converter.toRosMsg(depth_frame, msg);
            pub_depth.publish(msg);
            ++i;
        }

        std::this_thread::sleep_for (std::chrono::milliseconds(1));


        ros::spinOnce();
    }

    
    return 0;
/*
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stereo_pipeline.getExposedImageStreams();
    
    std::vector<ros::Publisher> imgPubList;
    std::vector<std::string> frameNames;
    
    // this part would be removed once we have calibration-api
    std::string left_uri = camera_param_uri +"/" + "left.yaml";
    std::string right_uri = camera_param_uri + "/" + "right.yaml";
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";

    // dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter leftConverter("camera", true);

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
        imageDataQueues[0],
        node_handle, 
        std::string("left/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
            &leftConverter, 
            std::placeholders::_1, 
            std::placeholders::_2), 
        queue_length,
        left_uri,
        "left");

    leftPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter rightconverter("camera", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
        imageDataQueues[1],
        node_handle, 
        std::string("right/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
            &rightconverter, 
            std::placeholders::_1, 
            std::placeholders::_2), 
        queue_length,
        right_uri,
        "right");

    rightPublish.addPubisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
        imageDataQueues[2],
        node_handle, 
        std::string("depth/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
            &rightconverter, // since the converter has the same frame name
                            // and image type is also same we can reuse it
            std::placeholders::_1, 
            std::placeholders::_2) , 
        queue_length,
        stereo_uri,
        "depth");

    depthPublish.addPubisherCallback();

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    ROS_INFO("spinning stereo_node main.cpp");
    ros::spin();

    ROS_INFO("exiting stereo_node main.cpp");
    return 0;
*/
}


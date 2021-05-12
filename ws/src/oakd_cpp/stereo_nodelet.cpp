#include "ros/ros.h"
#include <ros/console.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include "stereo_pipeline.hpp"
#include <functional>

#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace oakd_cpp {

class StereoNodelet : public nodelet::Nodelet
{
    public:
        virtual void onInit(){

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

            StereoPipeline stereo_pipeline;
            stereo_pipeline.initDepthaiDev();
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

            ros::spin();

            return ;
        }
};

PLUGINLIB_EXPORT_CLASS(oakd_cpp::StereoNodelet, nodelet::Nodelet)

}   // namespace oakd_cpp





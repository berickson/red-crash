#include "ros/ros.h"
#include <ros/console.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include "stereo_pipeline.hpp"
#include <functional>

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace oakd_cpp{


 class StereoNodelet : public nodelet::Nodelet
{
    public:
        virtual void onInit(){

            ros::NodeHandle pnh("~");
            ROS_ERROR("***** StereoNodelet::onInit");
            
            std::string deviceName;
            std::string camera_param_uri;
            int bad_params = 0;
            int queue_length = 1;

            if(!pnh.getParam("camera_name", deviceName))
            {
                ROS_ERROR("Couldn't getParam camera_name");
                throw std::runtime_error("Couldn't getParam camera_name");
            }
            if(!pnh.getParam("camera_param_uri", camera_param_uri))
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
            ROS_WARN_STREAM("loading " << left_uri);
        
            std::string right_uri = camera_param_uri + "/" + "right.yaml";
            
            std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
            

        
            dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(imageDataQueues[0],
                                                                                            pnh, 
                                                                                            std::string("left/image"),
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &converter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            queue_length,
                                                                                            left_uri,
                                                                                            "left");

            // bridgePublish.startPublisherThread();
            leftPublish.addPubisherCallback();

            dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(imageDataQueues[1],
                                                                                            pnh, 
                                                                                            std::string("right/image"),
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &rightconverter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            queue_length,
                                                                                            right_uri,
                                                                                            "right");

            rightPublish.addPubisherCallback();

            // dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame");
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(imageDataQueues[2],
                                                                                            pnh, 
                                                                                            std::string("stereo/depth"),
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &rightconverter, // since the converter has the same frame name
                                                                                                            // and image type is also same we can reuse it
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            queue_length,
                                                                                            stereo_uri,
                                                                                            "stereo");

            depthPublish.addPubisherCallback();

            // We can add the rectified frames also similar to these publishers. 
            // Left them out so that users can play with it by adding and removing

            ros::spin();

            return ;
        }
};

PLUGINLIB_EXPORT_CLASS(oakd_cpp::StereoNodelet, nodelet::Nodelet)


}   // namespace depthai_examples
// PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)




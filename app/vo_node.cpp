
// #include <gflags/gflags.h>
#include <stdexcept>

#include "myslam/visual_odometry.h"
#include "rclcpp/rclcpp.hpp" // ros2

#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using namespace message_filters;
using std::placeholders::_1;

class VONode : public rclcpp::Node
{
  public:

    VONode(const std::string & configPath)
    : Node("vo_node"){

    // get topic names from config file
    myslam::Config config;
    config.SetParameterFile(configPath);

    leftImageTopic = Config::Get<string>("cam0_topic");
    rightImageTopic = Config::Get<string>("cam1_topic");

    // init callbacks
    image0Sub(nh, leftImageTopic, 1000);
    image1Sub(nh, rightImageTopic, 1000);

    vo = std::make_shared<myslam::VisualOdometry>(configPath);
    assert(vo->Init() == true);

    // Synchronize the messages from the two image topics
    time_sync(MySyncPolicy(1000), image0Sub, image1Sub)
    time_sync.registerCallback(std::bind(&VONode::imageCallback, this, _1, _2));
    
    }

  private:

    myslam::VisualOdometry::Ptr vo;
    std::string leftImageTopic, rightImageTopic;
    message_filters::Subscriber<sensor_msgs::msg::Image> image0Sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> image1Sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> time_sync;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_0,
                         const sensor_msgs::msg::Image::SharedPtr img_1) const
    { 

      // Convert left_img_msg and right_img_msg to cv::Mat
      const cv::Mat raw_img_0(img_0->height, img_0->width, CV_8UC1,
                    const_cast<uint8_t*>(&img_0->data[0]), img_0->step);
      const cv::Mat raw_img_1(img_1->height, img_1->width, CV_8UC1,
                    const_cast<uint8_t*>(&img_1->data[0]), img_1->step);

      auto new_frame = Frame::CreateFrame();
      new_frame->left_img_ = raw_img_0;
      new_frame->right_img_ = raw_img_1;
      vo->Step(new_frame);

    }

};

int main(int argc, char * argv[])
{   
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;

    std::vector<std::string> allArgs;

    if (argc > 1) {
        allArgs.assign(argv + 1, argv + argc);
    }
    else throw std::runtime_error("Pass a config file.");

    std::string configPath(allArgs[0]);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VONode>(configPath));
    rclcpp::shutdown();

    return 0;
}


// #include <gflags/gflags.h>
#include <stdexcept>

#include "myslam/visual_odometry.h"
#include "rclcpp/rclcpp.hpp" // ros2

#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace message_filters;
using std::placeholders::_1;

namespace myslam {

class VONode : public rclcpp::Node
{
  public:

    VONode(std::string & configPath):

      Node("vo_node"),

      time_sync(ApproxPolicy(10), image0Sub, image1Sub)

    {

    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    // get topic names from config file
    if (Config::SetParameterFile(configPath) == false) {
        throw std::runtime_error("[ERROR] Forgot to pass node config file");
    }

    vo = std::make_shared<myslam::VisualOdometry>(configPath);
    assert(vo->Init() == true);

    leftImageTopic = Config::Get<std::string>("cam0_topic");
    rightImageTopic = Config::Get<std::string>("cam1_topic");

    // If you dont want to use init list, do:

    // init callbacks
    image0Sub.subscribe(this, leftImageTopic, rmw_qos_profile);
    image1Sub.subscribe(this, rightImageTopic, rmw_qos_profile);

    // Synchronize the messages from the two image topics
    time_sync.registerCallback(std::bind(&VONode::imagesCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    }

  private:

    myslam::VisualOdometry::Ptr vo;
    std::string leftImageTopic, rightImageTopic;

    message_filters::Subscriber<sensor_msgs::msg::Image> image0Sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> image1Sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproxPolicy;
    message_filters::Synchronizer<ApproxPolicy> time_sync;

    void imagesCallback(const std::shared_ptr<const sensor_msgs::msg::Image>& img_0,
                    const std::shared_ptr<const sensor_msgs::msg::Image>& img_1)
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

}

int main(int argc, char * argv[])
{   
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;

    std::vector<std::string> allArgs;

    if (argc > 1) {
        allArgs.assign(argv + 1, argv + argc);
    }
    else throw std::runtime_error("[ERROR] Pass a config file.");

    std::string configPath(allArgs[0]);

    rclcpp::init(argc, argv);

    LOG(INFO) << "Waiting on input frames...";

    rclcpp::spin(std::make_shared<myslam::VONode>(configPath));


    rclcpp::shutdown();

    return 0;
}
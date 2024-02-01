
#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
#include <stdexcept>

int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0; // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
    FLAGS_colorlogtostderr = 1;

    std::vector<std::string> allArgs;

    if (argc > 1) {
        allArgs.assign(argv + 1, argv + argc);
    }
    else throw std::runtime_error("Pass a config file.");

    std::string configPath(allArgs[0]);
    
    // Substitute smart pointer allocation (make shared) instead of nnew
    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(configPath));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}

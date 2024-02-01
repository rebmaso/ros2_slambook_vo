cmake_minimum_required(VERSION 3.8)
project(ros_slambk_vo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -std=c++14) # -Wpedantic -Wextra
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
# include_directories(${Sophus_INCLUDE_DIRS}) # BAD, REQUIRES MANUALLY LINKING FMT HERE

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# dbow
find_package(DBoW3 REQUIRED)
include_directories(${DBoW3_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        # ${Sophus_LIBRARIES} # BAD, REQUIRES MANUALLY LINKING FMT HERE
        Sophus::Sophus
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        ${DBoW3_LIBS}
        # fmt # ONLY NEED THIS IF INCLUDING SOPHUS THE OLD WAY
        )

enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include/ros_slambk_vo)
add_subdirectory(src)
add_subdirectory(app)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

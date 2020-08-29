/*-*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "flex_sync/live_sync.h"
#include "flex_sync/exact_sync.h"
#include "flex_sync/approximate_sync.h"

#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Temperature.h>

#include <vector>
#include <string>

using std::vector;
using std::string;

typedef sensor_msgs::Illuminance Msg0;
typedef sensor_msgs::Temperature Msg1;

struct Callback {
  void callback(const std::vector<Msg0::ConstPtr> &im,
                const std::vector<Msg1::ConstPtr> &ci) {
    std::cout << "callback!" << std::endl;
    callback_cnt++;
  }
  void reset() { callback_cnt = 0; }
  size_t callback_cnt{0};
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "live_sync_test");
  ros::NodeHandle pnh("~");

  try {
    vector<vector<string>> topics = {{"/left_tof/stream/1/mono8",
                                      "/right_tof/stream/1/mono8",
                                      "/t265/fisheye1/image_raw"},
                                     {"/left_tof/depth/camera_info",
                                      "/right_tof/depth/camera_info",
                                      "/t265/fisheye1/camera_info"}};
    Callback cb;
#if 1
    typedef flex_sync::LiveSync<flex_sync::ApproximateSync<Msg0, Msg1>> SyncT;
#else
    typedef flex_sync::LiveSync<flex_sync::ExactSync<Msg0, Msg1>> SyncT;
#endif
    SyncT::Callback cbf = std::bind(&Callback::callback, &cb,
                                    std::placeholders::_1,
                                    std::placeholders::_2);
    SyncT liveSync(pnh, topics, cbf);

    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
  return (0);
}


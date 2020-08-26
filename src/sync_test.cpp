/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "flex_sync/exact_sync.h"
#include "flex_sync/approximate_sync.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Temperature.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <vector>
#include <string>

using std::vector;
using std::string;

typedef sensor_msgs::Illuminance Msg0;
typedef sensor_msgs::Temperature Msg1;

// sneaky class to gain access to the signalMessage() method
// taken from the ros cookbook
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
public:
  void newMessage(const boost::shared_ptr<M const> &msg)  {
    message_filters::SimpleFilter<M>::signalMessage(msg);
  }
};

static int check_test(const std::string &testName,
                      size_t n1, size_t n2, size_t n_good) {
  if (n1 == n_good && n2 == n_good) {
    ROS_INFO_STREAM("PASSED  " << testName << " with "
                    << n_good << " messages");
    return (1);
  } else {
    ROS_ERROR_STREAM("FAILED " << testName << " with "
                     << n1 << " vs " << n2 << " vs "
                     << n_good << " messages");
  }
  return (0);
}

struct Callback1 {
  void callback(const std::vector<Msg0::ConstPtr> &im,
                const std::vector<Msg1::ConstPtr> &ci) {
    callback_cnt++;
  }
  void callback_mf(const Msg0::ConstPtr &im,
                   const Msg1::ConstPtr &ci) {
    callback_cnt++;
  }
  void reset() { callback_cnt = 0; }
  size_t callback_cnt{0};
};

struct Callback2 {
  void callback(const std::vector<Msg0::ConstPtr> &im,
                const std::vector<Msg1::ConstPtr> &temp) {
    if (im.size() != 3 || temp.size() != 3) {
      ROS_ERROR("bad callback2 size!");
      throw std::runtime_error("bad size!");
    }
    callback_cnt++;
  }
  void callback_mf(const Msg0::ConstPtr &im0,
                   const Msg0::ConstPtr &im1,
                   const Msg0::ConstPtr &im2,
                   const Msg1::ConstPtr &temp0,
                   const Msg1::ConstPtr &temp1,
                   const Msg1::ConstPtr &temp2) {
    callback_cnt++;
  }
  void reset() { callback_cnt = 0; }
  size_t callback_cnt{0};
};

static vector<string> flatten(const vector<vector<string>> &topics) {
  vector<string> flattened;
  for (const auto &tv: topics) {
    flattened.insert(flattened.end(), tv.begin(), tv.end());
  }
  return (flattened);
}

template<typename SyncType>
void run_test(SyncType *sync,
              const string &testName,
              const vector<vector<string>> &topics,
              const std::string &bagName) {
  rosbag::Bag bag;
  bag.open(bagName, rosbag::bagmode::Read);
  vector<string> bag_topics = flatten(topics);

  rosbag::View view(bag, rosbag::TopicQuery(bag_topics));

  for (rosbag::MessageInstance m: view) {
    Msg0::ConstPtr ill = m.instantiate<Msg0>();
    if (ill) {
      sync->process(m.getTopic(), ill);
    } else {
      Msg1::ConstPtr temp =
        m.instantiate<Msg1>();
      if (temp) {
        sync->process(m.getTopic(), temp);
      }
    }
  }
  bag.close();
}

template<typename SyncPolicy, typename MsgType0, typename MsgType1>
void do_playback(
  const std::string &bagName,
  const vector<vector<string>> &topics,
  const std::map<std::string, std::shared_ptr<BagSubscriber<MsgType0>>> &s0,
  const std::map<std::string, std::shared_ptr<BagSubscriber<MsgType1>>> &s1
  ) {
  const vector<string> bag_topics = flatten(topics);
  rosbag::Bag bag;
  bag.open(bagName, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(bag_topics));
  
  for (rosbag::MessageInstance m: view) {
    Msg0::ConstPtr ill = m.instantiate<Msg0>();
    if (ill) {
      const auto it = s0.find(m.getTopic());
      if (it != s0.end()) {
        it->second->newMessage(ill);
      }
    } else {
      Msg1::ConstPtr temp = m.instantiate<Msg1>();
      if (temp) {
        const auto it = s1.find(m.getTopic());
        if (it != s1.end()) {
          it->second->newMessage(temp);
        }
      }
    }
  }
  bag.close();
}


template<typename MsgType0, typename MsgType1>
static
std::pair<std::map<string, std::shared_ptr<BagSubscriber<MsgType0>>>,
          std::map<string, std::shared_ptr<BagSubscriber<MsgType1>>>>
make_subscribers(const vector<vector<string>> &topics) {
  std::pair<std::map<string, std::shared_ptr<BagSubscriber<MsgType0>>>,
            std::map<string, std::shared_ptr<BagSubscriber<MsgType1>>>>
    rv;
  for (const auto &topic: topics[0]) {
    rv.first[topic] = std::make_shared<BagSubscriber<MsgType0>>();
  }
  for (const auto &topic: topics[1]) {
    rv.second[topic] = std::make_shared<BagSubscriber<MsgType1>>();
  }
  return (rv);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "exact_sync_test");
  ros::NodeHandle pnh("~");
  std::string bagName;
  pnh.param<std::string>("bag", bagName, "test.bag");
  ROS_INFO_STREAM("using bag: " << bagName);

  vector<vector<string>> topics =
    {{"/left_tof/stream/1/mono8"},
     {"/left_tof/depth/camera_info"}};
  try {
    //
    // ------------ test 1
    //
    int n_good = 432;
    std::string testName = "test 1 (exact)";
    Callback1 cb1;
    const uint32_t qsize = 2;
    flex_sync::ExactSync<Msg0, Msg1>
      sync1(topics, std::bind(
             &Callback1::callback, &cb1, std::placeholders::_1,
             std::placeholders::_2), qsize);
    run_test(&sync1, testName, topics, bagName);
    size_t nsynced = cb1.callback_cnt;
    cb1.reset();
    {
      typedef message_filters::sync_policies::ExactTime<Msg0, Msg1> SyncPolicy;
      auto subs = make_subscribers<Msg0, Msg1>(topics);
      message_filters::Synchronizer<SyncPolicy>
        syncmf1(SyncPolicy(qsize), *subs.first[topics[0][0]],
                *subs.second[topics[1][0]]);
      syncmf1.registerCallback(
        std::bind(&Callback1::callback_mf, &cb1,
                  std::placeholders::_1, std::placeholders::_2));
      do_playback<SyncPolicy, Msg0, Msg1>(bagName, topics,
                                          subs.first, subs.second);
    }
    check_test(testName, cb1.callback_cnt, nsynced, n_good);
    cb1.reset();
    //
    // ------------ test 1a
    //
    n_good = 432;
    testName = "test 1 (approx)";
    flex_sync::ApproximateSync<Msg0, Msg1>
      sync1a(topics, std::bind(
             &Callback1::callback, &cb1, std::placeholders::_1,
             std::placeholders::_2), qsize);
    run_test(&sync1a, testName, topics, bagName);
    nsynced = cb1.callback_cnt;
    cb1.reset();
    {
      typedef message_filters::sync_policies::ApproximateTime<
        Msg0, Msg1> SyncPolicy1a;
      auto subs = make_subscribers<Msg0, Msg1>(topics);
      message_filters::Synchronizer<SyncPolicy1a>
        syncmf1a(SyncPolicy1a(qsize), *subs.first[topics[0][0]],
                 *subs.second[topics[1][0]]);
      syncmf1a.registerCallback(
        std::bind(&Callback1::callback_mf, &cb1,
                  std::placeholders::_1, std::placeholders::_2));
      do_playback<SyncPolicy1a, Msg0, Msg1>(bagName, topics,
                                            subs.first, subs.second);
    }
    check_test(testName, cb1.callback_cnt, nsynced, n_good);
    cb1.reset();
    //
    // ------------ test 2
    //
    topics = {{"/left_tof/stream/1/mono8",
               "/right_tof/stream/1/mono8",
               "/t265/fisheye1/image_raw"},
              {"/left_tof/depth/camera_info",
               "/right_tof/depth/camera_info",
               "/t265/fisheye1/camera_info"}};
    n_good = 219;
    testName = "test 2 (approx)";
    Callback2 cb2;
    flex_sync::ApproximateSync<Msg0, Msg1>
      sync2a(topics, std::bind(
               &Callback2::callback, &cb2, std::placeholders::_1,
               std::placeholders::_2), qsize);
    run_test(&sync2a, testName, topics, bagName);
    nsynced = cb2.callback_cnt;
    cb2.reset();
    {
      typedef message_filters::sync_policies::ApproximateTime<
        Msg0, Msg0, Msg0, Msg1, Msg1, Msg1> SyncPolicy2a;
      auto subs = make_subscribers<Msg0, Msg1>(topics);
      message_filters::Synchronizer<SyncPolicy2a>
        sync2mf(SyncPolicy2a(qsize),
                *subs.first[topics[0][0]],
                *subs.first[topics[0][1]],
                *subs.first[topics[0][2]],
                *subs.second[topics[1][0]],
                *subs.second[topics[1][1]],
                *subs.second[topics[1][2]]);
      sync2mf.registerCallback(
        std::bind(&Callback2::callback_mf, &cb2,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4,
                  std::placeholders::_5, std::placeholders::_6));
      do_playback<SyncPolicy2a, Msg0, Msg1>(bagName, topics,
                                            subs.first, subs.second);
    }
    check_test(testName, cb2.callback_cnt, nsynced, n_good);
    cb2.reset();
    ROS_INFO("done with all tests");
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
  return (0);
}


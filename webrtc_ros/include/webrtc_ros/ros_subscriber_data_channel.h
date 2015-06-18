#ifndef WEBRTC_ROS_ROS_SUBSCRIBER_DATA_CHANNEL_H_
#define WEBRTC_ROS_ROS_SUBSCRIBER_DATA_CHANNEL_H_

#include "talk/app/webrtc/datachannelinterface.h"
#include "webrtc/base/scoped_ref_ptr.h"
#include <ros/ros.h>
#include "topic_tools/shape_shifter.h"

namespace webrtc_ros
{

class RosSubscriberDataChannel : public webrtc::DataChannelObserver {
public:
  RosSubscriberDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface>& data_channel, ros::NodeHandle& nh, const std::string& topic);
  virtual ~RosSubscriberDataChannel();

  virtual void OnStateChange();
  virtual void OnMessage(const webrtc::DataBuffer& buffer);

private:
  DISALLOW_COPY_AND_ASSIGN(RosSubscriberDataChannel);

  struct packet_header_t {
    uint8_t type;
  };
  static_assert(sizeof(packet_header_t) == 1, "sizes do not match");

  const static uint8_t MESSAGE_PACKET_TYPE;
  const static uint8_t DEFINITION_PACKET_TYPE;
  const static uint8_t FRAGMENT_PACKET_TYPE;
  const static uint8_t FRAGMENT_END_PACKET_TYPE;
  const static uint8_t ZLIB_PACKET_TYPE;

  void messageCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);

  rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel_;
  ros::NodeHandle nh_;
  ros::ServiceClient message_info_client_;
  ros::Subscriber sub_;

  std::string last_msg_type_;
};

}

#endif

#include "webrtc_ros/ros_subscriber_data_channel.h"
#include "webrtc_ros/GetMessageInfo.h"
#include "webrtc/base/json.h"
#include "zlib.h"

namespace webrtc_ros
{

RosSubscriberDataChannel::RosSubscriberDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface>& data_channel, ros::NodeHandle& nh, const std::string& topic)
  : data_channel_(data_channel), nh_(nh)
{
  data_channel_->RegisterObserver(this);
  message_info_client_ = nh_.serviceClient<webrtc_ros::GetMessageInfo>("get_message_info");

  ros::SubscribeOptions opts;
  opts.init<topic_tools::ShapeShifter>(topic, 1, boost::bind(&RosSubscriberDataChannel::messageCallback, this, _1));
  sub_ = nh_.subscribe(opts);
}

RosSubscriberDataChannel::~RosSubscriberDataChannel()
{
}

const uint8_t RosSubscriberDataChannel::DEFINITION_PACKET_TYPE = 0;
const uint8_t RosSubscriberDataChannel::MESSAGE_PACKET_TYPE = 1;
const uint8_t RosSubscriberDataChannel::FRAGMENT_PACKET_TYPE = 2;
const uint8_t RosSubscriberDataChannel::FRAGMENT_END_PACKET_TYPE = 3;
const uint8_t RosSubscriberDataChannel::ZLIB_PACKET_TYPE = 4;

void RosSubscriberDataChannel::messageCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
  if(last_msg_type_ != msg->getDataType()) {
    last_msg_type_ = msg->getDataType();

    webrtc_ros::GetMessageInfo::Request req;
    req.type = msg->getDataType();
    webrtc_ros::GetMessageInfo::Response res;
    if(message_info_client_.call(req, res)) {
      Json::Value message_info_json;
      message_info_json["type"] = msg->getDataType();
      Json::Value infos_array_json(Json::arrayValue);
      for(auto& info : res.infos) {
	Json::Value info_json;
	info_json["type"] = info.type;
	info_json["md5sum"] = info.md5sum;
	info_json["definition"] = info.definition;

	Json::Value fields_array_json(Json::arrayValue);
	for(auto& field : info.fields) {
	  Json::Value field_json;
	  field_json["name"] = field.name;
	  field_json["type"] = field.type;
	  fields_array_json.append(field_json);
	}
	info_json["fields"] = fields_array_json;
	infos_array_json.append(info_json);
      }
      message_info_json["infos"] = infos_array_json;

      Json::FastWriter writer;
      std::string message_info_json_str = writer.write(message_info_json);

      rtc::Buffer buffer(sizeof(packet_header_t));
      packet_header_t *header = (packet_header_t*)buffer.data();
      header->type = DEFINITION_PACKET_TYPE;

      buffer.AppendData(message_info_json_str.data(), message_info_json_str.length());
      data_channel_->Send(webrtc::DataBuffer(buffer, true));
    }
  }

  size_t length = ros::serialization::serializationLength(*msg);
  rtc::Buffer raw_buffer(sizeof(packet_header_t) + length);
  packet_header_t *raw_header = (packet_header_t*)raw_buffer.data();
  raw_header->type = MESSAGE_PACKET_TYPE;

  ros::serialization::OStream ostream((uint8_t*)raw_buffer.data() + sizeof(packet_header_t), length);
  ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

  rtc::Buffer buffer(10000);// = raw_buffer;
  packet_header_t *header = (packet_header_t*)buffer.data();
  header->type = ZLIB_PACKET_TYPE;

  z_stream strm;
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  deflateInit(&strm, Z_DEFAULT_COMPRESSION);
  strm.avail_in = raw_buffer.length();
  strm.next_in = (Bytef*)raw_buffer.data();
  strm.avail_out = buffer.size()-sizeof(packet_header_t);
  strm.next_out = (Bytef*)buffer.data() + sizeof(packet_header_t);
  deflate(&strm, Z_FINISH);
  deflateEnd(&strm);

  buffer.SetSize(buffer.size() - strm.avail_out);

  const static size_t MAX_PACKET_SIZE = 1000;
  const static size_t MAX_FRAGMENT_SIZE = MAX_PACKET_SIZE - sizeof(packet_header_t);

  if(buffer.size() > MAX_PACKET_SIZE) {
    for(size_t pos = 0; pos < buffer.size(); pos += MAX_FRAGMENT_SIZE) {
      size_t fragment_size = std::min(MAX_FRAGMENT_SIZE, buffer.size() - pos);

      rtc::Buffer fragment_buffer(sizeof(packet_header_t) + fragment_size);
      packet_header_t *fragment_header = (packet_header_t*)fragment_buffer.data();

      if(pos + fragment_size < buffer.size()) {
	fragment_header->type = FRAGMENT_PACKET_TYPE;
      }
      else {
	fragment_header->type = FRAGMENT_END_PACKET_TYPE;
      }
      memcpy(fragment_buffer.data() + sizeof(packet_header_t), buffer.data() + pos, fragment_size);

      data_channel_->Send(webrtc::DataBuffer(fragment_buffer, true));
    }

  }
  else {
    data_channel_->Send(webrtc::DataBuffer(buffer, true));
  }
}

void RosSubscriberDataChannel::OnStateChange()
{
}

void RosSubscriberDataChannel::OnMessage(const webrtc::DataBuffer& buffer)
{
  std::string message((const char*)buffer.data.data(), buffer.size());
  ROS_ERROR_STREAM("Got message["<<buffer.size()<<"]: " << message);
}

}

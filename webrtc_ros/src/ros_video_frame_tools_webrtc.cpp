#include "webrtc_ros/ros_video_frame_tools.h"
#include "talk/media/base/videocapturer.h"

namespace webrtc_ros
{

cricket::CapturedFrame *__createCricketCapturedFrame(int width, int height, uint32_t data_size, void *data) {
  cricket::CapturedFrame *frame = new cricket::CapturedFrame;
  frame->width = width;
  frame->height = height;
  frame->fourcc = cricket::FOURCC_I420;
  frame->data_size = data_size;
  frame->data = data;
  return frame;
}

}

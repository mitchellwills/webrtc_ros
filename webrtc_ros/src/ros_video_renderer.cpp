#include "webrtc_ros/ros_video_renderer.h"
#include "webrtc_ros/ros_video_frame_tools.h"
#include <talk/media/base/videoframe.h>

namespace webrtc_ros
{


RosVideoRenderer::RosVideoRenderer(image_transport::ImageTransport it, const std::string& topic)
  : it_(it), topic_(topic)
{
  pub_ = it_.advertise(topic_, 1);
}


RosVideoRenderer::~RosVideoRenderer()
{
}

void RosVideoRenderer::SetSize(int width, int height)
{
  ROS_DEBUG("Render SetSize %d,%d", width, height);
}

void RosVideoRenderer::RenderFrame(const cricket::VideoFrame* frame)
{
  pub_.publish(createRosImageFromVideoFrameData(frame->GetWidth(), frame->GetHeight(),
						frame->GetYPlane(), frame->GetYPitch(),
						frame->GetUPlane(), frame->GetUPitch(),
						frame->GetVPlane(), frame->GetVPitch(),
						ros::Time::now()));
}

}

#ifndef WEBRTC_ROS_ROS_VIDEO_FRAME_TOOLS_H_
#define WEBRTC_ROS_ROS_VIDEO_FRAME_TOOLS_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// This is needed because webrtc and opencv have conflicting typedefs in
// their headers so the opencv code must be in a seperate file from the webrtc
// code.

namespace cricket {
  class CapturedFrame;
}

namespace webrtc_ros
{

sensor_msgs::ImagePtr createRosImageFromVideoFrameData(size_t width, size_t height,
						       const uint8_t *yPlane, int32_t yPitch,
						       const uint8_t *uPlane, int32_t uPitch,
						       const uint8_t *vPlane, int32_t vPitch,
						       const ros::Time& stamp);

cricket::CapturedFrame *createCapturedFrameFromImage(const sensor_msgs::ImageConstPtr& msg);
cricket::CapturedFrame *__createCricketCapturedFrame(int width, int height, uint32_t data_size, void *data);

}


#endif

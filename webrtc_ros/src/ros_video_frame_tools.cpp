#include "webrtc_ros/ros_video_frame_tools.h"
#include <libyuv/convert_from.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace webrtc_ros
{

sensor_msgs::ImagePtr createRosImageFromVideoFrameData(size_t width, size_t height,
						       const uint8_t *yPlane, int32_t yPitch,
						       const uint8_t *uPlane, int32_t uPitch,
						       const uint8_t *vPlane, int32_t vPitch,
						       const ros::Time& stamp) {
  std_msgs::Header header;
  header.stamp = stamp;

  cv::Mat bgra(height, width, CV_8UC4);
  // The ARGB function in libyuv appears to output BGRA...
  libyuv::I420ToARGB(yPlane, yPitch,
		     uPlane, uPitch,
		     vPlane, vPitch,
                     bgra.data, bgra.step, width, height);


  cv_bridge::CvImage image(header, "bgra8", bgra);
  return image.toImageMsg();
}

cricket::CapturedFrame *createCapturedFrameFromImage(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat bgr;
  if (msg->encoding.find("F") != std::string::npos)
  {
    // scale floating point images
    cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if (max_val > 0)
    {
      float_image *= (255 / max_val);
    }
    cv::Mat orig;
    float_image.convertTo(orig, CV_8U);
    bgr = cv::Mat(bgr.rows, bgr.cols, CV_8UC3);
    cv::cvtColor(orig, bgr, CV_GRAY2BGR);
  }
  else
  {
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }

  cv::Mat yuv(bgr.rows, bgr.cols, CV_8UC4);
  cv::cvtColor(bgr, yuv, CV_BGR2YUV_I420);

  return __createCricketCapturedFrame(bgr.cols, bgr.rows, yuv.rows * yuv.step, yuv.data);
}

}

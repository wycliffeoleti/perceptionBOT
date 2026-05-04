#include "perception_bot_perception/hsv_detector.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <std_msgs/msg/header.hpp>

namespace {

cv::Mat make_canvas() {
  return cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // black BG
}

// Draws a filled BGR rectangle.
void draw_rect(cv::Mat &img, const cv::Rect &r, const cv::Scalar &bgr) {
  cv::rectangle(img, r, bgr, cv::FILLED);
}

}  // namespace

TEST(HsvDetector, EmptyImageProducesNoDetections) {
  perception_bot::HsvDetector det;
  cv::Mat empty;
  std_msgs::msg::Header h;
  EXPECT_TRUE(det.detect(empty, h).empty());
}

TEST(HsvDetector, DetectsRedBlueGreenBlobs) {
  cv::Mat img = make_canvas();
  draw_rect(img, cv::Rect(50, 50, 60, 60), cv::Scalar(0, 0, 255));     // red
  draw_rect(img, cv::Rect(200, 60, 60, 60), cv::Scalar(255, 0, 0));    // blue
  draw_rect(img, cv::Rect(380, 70, 30, 80), cv::Scalar(0, 200, 0));    // green tall

  perception_bot::HsvDetector det;
  std_msgs::msg::Header h;
  h.frame_id = "camera_optical_frame";
  auto dets = det.detect(img, h);

  ASSERT_EQ(dets.size(), 3u);

  bool saw_red = false, saw_blue = false, saw_green = false;
  for (const auto &d : dets) {
    if (d.class_name.rfind("red_", 0) == 0) saw_red = true;
    if (d.class_name.rfind("blue_", 0) == 0) saw_blue = true;
    if (d.class_name.rfind("green_", 0) == 0) saw_green = true;
    EXPECT_EQ(d.header.frame_id, "camera_optical_frame");
    EXPECT_GT(d.bbox.width, 0u);
    EXPECT_GT(d.bbox.height, 0u);
  }
  EXPECT_TRUE(saw_red);
  EXPECT_TRUE(saw_blue);
  EXPECT_TRUE(saw_green);
}

TEST(HsvDetector, ClassifiesAspectRatio) {
  cv::Mat img = make_canvas();
  // Square red blob → cube
  draw_rect(img, cv::Rect(50, 50, 60, 60), cv::Scalar(0, 0, 255));
  // Tall green blob → cylinder
  draw_rect(img, cv::Rect(380, 70, 30, 80), cv::Scalar(0, 200, 0));

  perception_bot::HsvDetector det;
  std_msgs::msg::Header h;
  auto dets = det.detect(img, h);

  for (const auto &d : dets) {
    if (d.class_name.rfind("red_", 0) == 0) {
      EXPECT_EQ(d.class_name, "red_cube");
    }
    if (d.class_name.rfind("green_", 0) == 0) {
      EXPECT_EQ(d.class_name, "green_cylinder");
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

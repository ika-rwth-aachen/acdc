#include <gtest/gtest.h>

#include <GeographicLib/UTMUPS.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace localization {

/**
 * @brief Get the UTM Position defined by the given latitude and longitude coordinates
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * 
 * @param[in] latitude latitude coordinate in decimal degree
 * @param[in] longitude longitude coordinate in decimal degree
 * @param[out] geometry_msgs::msg::PointStamped indicating the position in the utm system
 * @return bool indicating if projection was succesful
*/
bool projectToUTM(const double& latitude, const double& longitude, geometry_msgs::msg::PointStamped& utm_point)
{
  try {
    // PASTE TASK 2 CODE HERE
 



    // return true if succesful
    return false;
    // PASTE TASK 2 CODE HERE
  } catch (GeographicLib::GeographicErr& e) {
    std::cout << "Tranformation from WGS84 to UTM failed: " << e.what() << std::endl;
    return false;
  }
}

TEST(TestGNSSLocalization, test) {
  double latitude = 50.787467;
  double longitude = 6.046498;
  geometry_msgs::msg::PointStamped utm_point;
  EXPECT_EQ(true, projectToUTM(latitude, longitude, utm_point));
  EXPECT_NEAR(291827.02, utm_point.point.x, 1e-2);
  EXPECT_NEAR(5630349.72, utm_point.point.y, 1e-2);
  EXPECT_EQ("utm", utm_point.header.frame_id);
}

} // namespace localization

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

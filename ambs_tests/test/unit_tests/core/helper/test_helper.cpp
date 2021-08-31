#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "ambs_core/ambs_helper/helper.h"

class TestHelper : public testing::Test
{
public:
  TestHelper() {}
  const std::vector<double> data_list_1{1.1, 2.2, 3.3, 4.4, 5.5};
  const double avg_dl_1 = 3.3;
  const double std_dl_1 = 1.5556349186104046;
  const double maxdev_dl_1 = 2.2;
};

TEST_F(TestHelper, test_mean)
{
  EXPECT_DOUBLE_EQ(ambs_helper::getMean(data_list_1), avg_dl_1);
}

TEST_F(TestHelper, test_stdev)
{
  EXPECT_DOUBLE_EQ(ambs_helper::getStandardDeviation(data_list_1), std_dl_1);
  EXPECT_NEAR(ambs_helper::getStandardDeviation(data_list_1), std_dl_1, 0.01);
}

TEST_F(TestHelper, test_maxdev)
{
  EXPECT_DOUBLE_EQ(ambs_helper::getMaxDeviation(data_list_1), maxdev_dl_1);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_float_param_comp");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}


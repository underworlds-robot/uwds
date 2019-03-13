#include <ros/ros.h>
#include <uwds/uwds.h>
#include <gtest/gtest.h>

#define PROPAGATION_TIME 0.2
#define SETUP_TIME 0.5
#define WAIT_FOR(X) ros::Duration(X).sleep()

TEST(Underworlds, push3DMesh)
{
  server = Underworlds(boost::make_shared<ros::NodeHandle>());
  WAIT_FOR(SETUP_TIME);
  ctx = UnderworldsProxy(boost::make_shared<ros::NodeHandle>(), "client_test", PROVIDER);
  ctx.worlds()["env"].pushSceneFrom3DFile(argv[1]);
  WAIT_FOR(PROPAGATION_TIME);
  TEST_EXPRESSION(ctx.worlds()["env"].size > 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "client_test");
  NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>()
  return RUN_ALL_TESTS();
}

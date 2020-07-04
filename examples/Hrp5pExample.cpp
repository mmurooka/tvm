/* Author: Masaki Murooka */

#include <tvm/Robot.h>
#include <tvm/robot/internal/GeometricContactFunction.h>
#include <tvm/robot/internal/DynamicFunction.h>
#include <tvm/robot/CollisionFunction.h>
#include <tvm/robot/CoMFunction.h>
#include <tvm/robot/CoMInConvexFunction.h>
#include <tvm/robot/ConvexHull.h>
#include <tvm/robot/JointsSelector.h>
#include <tvm/robot/OrientationFunction.h>
#include <tvm/robot/PositionFunction.h>
#include <tvm/robot/PostureFunction.h>
#include <tvm/robot/utils.h>
#include <tvm/Task.h>

#include <tvm/Clock.h>
#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/function/IdentityFunction.h>
#include <tvm/hint/Substitution.h>
#include <tvm/scheme/WeightedLeastSquares.h>
#include <tvm/solver/defaultLeastSquareSolver.h>
#include <tvm/solver/QuadprogLeastSquareSolver.h>
#include <tvm/task_dynamics/None.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>
#include <tvm/task_dynamics/VelocityDamper.h>
#include <tvm/utils/sch.h>

#include <RBDyn/parsers/urdf.h>

#include <RBDyn/ID.h>

#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include "Hrp5pSetting.h"


class Hrp5pExample
{
 public:
  Hrp5pExample():
      clock_(std::make_shared<tvm::Clock>(dt_))
  {
    setupRobot();
  }

  void run()
  {
    publishRobotState(*robot_);
  }

 protected:
  void setupRobot()
  {
    // setup robot
    std::string robot_name = "HRP5P";
    std::string robot_urdf = ros::package::getPath("hrp5_p_description") + "/urdf/hrp5_p_planH.urdf";

    robot_ = tvm::robot::fromURDF(
        *clock_,
        robot_name,
        robot_urdf,
        false,
        hrp5p_filtered_link_names,
        hrp5p_initial_q);

    // setup env
    std::string env_urdf = ros::package::getPath("mc_env_description") + "/urdf/ground.urdf";

    env_ = tvm::robot::fromURDF(
        *clock_,
        "env",
        env_urdf,
        true,
        {},
        {});
  }

  void publishRobotState(const tvm::Robot& robot)
  {
    for (const auto& joint : robot.mb().joints()) {
      if (joint.dof() == 1) {
        int joint_idx = robot.mb().jointIndexByName(joint.name());
        double joint_pos = robot.mbc().q[joint_idx][0];
        printf("%s: %lf\n", joint.name().c_str(), joint_pos);
      }
    }
  }

  double dt_ = 0.005; // [sec]

  tvm::ControlProblem pb_;
  std::shared_ptr<tvm::Clock> clock_;

  tvm::RobotPtr robot_;
  tvm::RobotPtr env_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Hrp5pExample");

  Hrp5pExample example;

  example.run();
}

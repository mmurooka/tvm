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
#include <geometry_msgs/Transform.h>
#include <moveit_msgs/DisplayRobotState.h>

#include "Hrp5pSetting.h"


class Hrp5pExample
{
 public:
  Hrp5pExample():
      clock_(std::make_shared<tvm::Clock>(dt_))
  {
    setupRobot();
    setupTask();
    setupRos();
  }

  void run()
  {
    solve();
  }

 protected:
  void setupRobot()
  {
    // setup robot
    robot_ = tvm::robot::fromURDF(
        *clock_,
        "HRP5P",
        ros::package::getPath("hrp5_p_description") + "/urdf/hrp5_p_planH.urdf",
        false,
        hrp5p_filtered_link_names,
        hrp5p_initial_q);

    env_ = tvm::robot::fromURDF(
        *clock_,
        "env",
        ros::package::getPath("mc_env_description") + "/urdf/ground.urdf",
        true,
        {},
        {});

    // setup frame
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "LeftFootFrame",
            robot_,
            "Lleg_Link5",
            sva::PTransformd{Eigen::Vector3d(0, 0, -0.102)}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "RightFootFrame",
            robot_,
            "Rleg_Link5",
            sva::PTransformd{Eigen::Vector3d(0, 0, -0.102)}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "GroundFrame",
            env_,
            "ground",
            sva::PTransformd::Identity()));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "LeftHand",
            robot_,
            "Lhand_Link0_Plan2",
            sva::PTransformd{Eigen::Vector3d(0, 0, -0.104)}));
    pushToFrameMap(
        std::make_shared<tvm::robot::Frame>(
            "RightHand",
            robot_,
            "Rhand_Link0_Plan2",
            sva::PTransformd{Eigen::Vector3d(0, 0, -0.104)}));

    // setup contact
    pushToContactMap(
        "LeftFootGround",
        std::make_shared<tvm::robot::Contact>(
            frame_map_["LeftFootFrame"], frame_map_["GroundFrame"], std::vector<sva::PTransformd>{
              {Eigen::Vector3d(0.1093074306845665, -0.06831501424312592, 0.)},
              {Eigen::Vector3d(0.10640743374824524, 0.06836499273777008, 0.)},
              {Eigen::Vector3d(-0.10778241604566574, 0.06897497922182083, 0.)},
              {Eigen::Vector3d(-0.1079324409365654, -0.069024957716465, 0.)}
            })) ;
    pushToContactMap(
        "RightFootGround",
        std::make_shared<tvm::robot::Contact>(
            frame_map_["RightFootFrame"], frame_map_["GroundFrame"], std::vector<sva::PTransformd>{
              {Eigen::Vector3d(-0.1079324409365654, 0.069024957716465, 0.)},
              {Eigen::Vector3d(-0.10778241604566574, -0.06897497922182083, 0.)},
              {Eigen::Vector3d(0.10640743374824524, -0.06836499273777008, 0.)},
              {Eigen::Vector3d(0.1093074306845665, 0.06831501424312592, 0.)}
            }));
  }

  void setupTask()
  {
    // define function
    auto left_foot_contact_fn = std::make_shared<tvm::robot::internal::GeometricContactFunction>(
        contact_map_["LeftFootGround"], Eigen::Matrix6d::Identity());
    auto right_foot_contact_fn = std::make_shared<tvm::robot::internal::GeometricContactFunction>(
        contact_map_["RightFootGround"], Eigen::Matrix6d::Identity());
    auto posture_fn = std::make_shared<tvm::robot::PostureFunction>(robot_);

    left_hand_ori_fn_ = std::make_shared<tvm::robot::OrientationFunction>(frame_map_["LeftHand"]);
    left_hand_pos_fn_ = std::make_shared<tvm::robot::PositionFunction>(frame_map_["LeftHand"]);

    // add task to problem
    pb_.add(left_foot_contact_fn == 0.,
            tvm::task_dynamics::PD(1.),
      {tvm::requirements::PriorityLevel(0)});
    pb_.add(right_foot_contact_fn == 0.,
            tvm::task_dynamics::PD(1.),
      {tvm::requirements::PriorityLevel(0)});
    pb_.add(posture_fn == 0.,
            tvm::task_dynamics::PD(1.),
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(1.)});
    pb_.add(left_hand_ori_fn_ == 0.,
            tvm::task_dynamics::PD(2.),
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(10.)});
    pb_.add(left_hand_pos_fn_ == 0.,
            tvm::task_dynamics::PD(1.),
      {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(10.)});

    // set bounds
    pb_.add(robot_->lQBound() <= robot_->qJoints() <= robot_->uQBound(),
            tvm::task_dynamics::VelocityDamper(dt_, {0.01, 0.001, 0}, tvm::constant::big_number),
            {tvm::requirements::PriorityLevel(0)});
    // pb_.add(robot_->lTauBound() <= robot_->tau() <= robot_->uTauBound(),
    //         tvm::task_dynamics::None(),
    //         {tvm::requirements::PriorityLevel(0)});
  }

  void setupRos()
  {
    robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);
  }

  void solve()
  {
    tvm::LinearizedControlProblem lpb(pb_);

    tvm::scheme::WeightedLeastSquares solver(tvm::solver::DefaultLSSolverOptions{});

    int loop_num = 10000;
    ros::Rate rate(static_cast<int>(1.0 / dt_));
    std::cout << "Will run solver for " << loop_num << " iterations" << std::endl;

    // loop
    for (int i = 0; i < loop_num; ++i) {
      // update target
      left_hand_ori_fn_->orientation(sva::RotY(-tvm::constant::pi/2));
      left_hand_pos_fn_->position(Eigen::Vector3d{0.5, 0.3, 1.0+0.2*std::sin(i/100.0)});
      // left_hand_pos_fn_->position(left_hand_pos_fn_->position() + Eigen::Vector3d{0.3, -0.1, 0.2});

      // solve
      bool res = solver.solve(lpb);
      if (!res) {
        std::cerr << "Solver failed" << std::endl;
        break;
      }

      // integrate
      clock_->advance();

      // process ROS
      publishRobotState(*robot_);
      ros::spinOnce();
      rate.sleep();
    }
  }

  void publishRobotState(const tvm::Robot& robot)
  {
    moveit_msgs::DisplayRobotState robot_state_msg;

    // set joint position
    for (const auto& joint : robot.mb().joints()) {
      if (joint.dof() == 1) {
        int joint_idx = robot.mb().jointIndexByName(joint.name());
        double joint_pos = robot.mbc().q[joint_idx][0];

        // printf("%s: %lf\n", joint.name().c_str(), joint_pos);
        robot_state_msg.state.joint_state.name.push_back(joint.name());
        robot_state_msg.state.joint_state.position.push_back(joint_pos);
      }
    }

    // set root position
    const std::vector<double>& root_q = robot.mbc().q[0];
    robot_state_msg.state.multi_dof_joint_state.header.frame_id = "world";
    geometry_msgs::Transform root_trans_msg;
    root_trans_msg.rotation.w = root_q[0];
    root_trans_msg.rotation.x = root_q[1];
    root_trans_msg.rotation.y = root_q[2];
    root_trans_msg.rotation.z = root_q[3];
    root_trans_msg.translation.x = root_q[4];
    root_trans_msg.translation.y = root_q[5];
    root_trans_msg.translation.z = root_q[6];
    robot_state_msg.state.multi_dof_joint_state.transforms.push_back(root_trans_msg);
    robot_state_msg.state.multi_dof_joint_state.joint_names.push_back("world_joint");

    // publish
    robot_state_pub_.publish(robot_state_msg);
  }

  void pushToFrameMap(const std::shared_ptr<tvm::robot::Frame>& frame)
  {
    frame_map_[frame->name()] = frame;
  }

  void pushToContactMap(const std::string& name,
                        const std::shared_ptr<tvm::robot::Contact>& contact)
  {
    contact_map_[name] = contact;
  }

  double dt_ = 0.005; // [sec]

  tvm::ControlProblem pb_;
  std::shared_ptr<tvm::Clock> clock_;

  tvm::RobotPtr robot_;
  tvm::RobotPtr env_;

  std::map<std::string, std::shared_ptr<tvm::robot::Frame> > frame_map_;
  std::map<std::string, std::shared_ptr<tvm::robot::Contact> > contact_map_;

  std::shared_ptr<tvm::robot::OrientationFunction> left_hand_ori_fn_;
  std::shared_ptr<tvm::robot::PositionFunction> left_hand_pos_fn_;

  ros::NodeHandle nh_;
  ros::Publisher robot_state_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Hrp5pExample");

  Hrp5pExample example;

  example.run();
}

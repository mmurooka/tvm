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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Hrp5pExample");

  tvm::ControlProblem pb;
}

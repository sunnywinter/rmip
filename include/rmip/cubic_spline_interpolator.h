#ifndef PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H
#define PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include "ros/assert.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

namespace path_smoothing
{

  class CubicSplineInterpolator
      {
        public:

      CubicSplineInterpolator(
        double pointsPerUnit = 5.0,
        unsigned int skipPoints = 0,
        bool useEndConditions = true,
        bool useMiddleConditions = true);

      CubicSplineInterpolator(std::string name);

      ~CubicSplineInterpolator();

      void loadRoadMap(const std::string fileName, geometry_msgs::PoseArray& path);

      void interpolatePath(
        const geometry_msgs::PoseArray& path, geometry_msgs::PoseArray& smoothedPath);

      void interpolatePath(
        const std::vector<geometry_msgs::Pose>& path,
        std::vector<geometry_msgs::Pose>& smoothedPath);

      void interpolatePoint(
        const std::vector<geometry_msgs::Pose>& path,
        const std::vector<double>& cummulativeDistances,
        geometry_msgs::Pose& point,
        double pointCummDist);

      void calcCummulativeDistances(
        const std::vector<geometry_msgs::Pose> path,
        std::vector<double>& cummulativeDistances);

      double calcTotalDistance(const std::vector<geometry_msgs::Pose>& path);

      double calcDistance(
        const std::vector<geometry_msgs::Pose>& path,
        unsigned int idx);

      double calcAlphaCoeff(
        const std::vector<geometry_msgs::Pose> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcBetaCoeff(
        const std::vector<geometry_msgs::Pose> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcGammaCoeff(
        const std::vector<geometry_msgs::Pose> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcDeltaCoeff(
        const std::vector<geometry_msgs::Pose> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcRelativeDistance(
        const std::vector<double>& cummulativeDistances,
        unsigned int idx,
        double input);

      void calcPointGradient(
        const std::vector<geometry_msgs::Pose>& path,
        const std::vector<double>& cummulativeDistances,
        unsigned int idx, std::vector<double>& gradient);

      unsigned int findGroup(
        const std::vector<double>& cummulativeDistances,
        double pointCummDist);

      double getPointsPerUnit() {return pointsPerUnit_;}
      unsigned int skipPoints() {return skipPoints_;}
      bool getUseEndConditions() {return useEndConditions_;}
      bool getUseMiddleConditions() {return useMiddleConditions_;}

      void setPointsPerUnit(double ppu) {pointsPerUnit_ = ppu;}
      void setSkipPoints(unsigned int sp) {skipPoints_ = sp;}
      void setUseEndConditions(bool uec) {useEndConditions_ = uec;}
      void setUseMiddleConditions(bool umc) {useMiddleConditions_ = umc;}

    private:
      double pointsPerUnit_;
      unsigned int skipPoints_;
      bool useEndConditions_;
      bool useMiddleConditions_;
  };

}  // namespace path_smoothing

#endif  // PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H

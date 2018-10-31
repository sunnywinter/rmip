#include "rmip/cubic_spline_interpolator.h"
#include <tf/tf.h>

namespace path_smoothing
{

  CubicSplineInterpolator::CubicSplineInterpolator(
    double pointsPerUnit,
    unsigned int skipPoints,
    bool useEndConditions,
    bool useMiddleConditions)
    :
      pointsPerUnit_(pointsPerUnit),
      skipPoints_(skipPoints),
      useEndConditions_(useEndConditions),
      useMiddleConditions_(useMiddleConditions)
  {
  }

  CubicSplineInterpolator::CubicSplineInterpolator(std::string name)
  {
    ros::NodeHandle pnh("~/" + name);

    pnh.param("points_per_unit", pointsPerUnit_, 5.0);
    pnh.param<bool>("use_end_conditions", useEndConditions_, true);
    pnh.param<bool>("use_middle_conditions", useMiddleConditions_, true);

    int skipPoints;
    pnh.param("skip_points", skipPoints, 0);
    skipPoints_ = abs(skipPoints);
  }


  CubicSplineInterpolator::~CubicSplineInterpolator()
  {
  }

void CubicSplineInterpolator::loadRoadMap(const std::string fileName, geometry_msgs::PoseArray& path){
  std::string line;
  std::ifstream fs;
  uint32_t roadId = 0;
  uint32_t roadType = 0;
  uint32_t parameter2 = 0;
  uint32_t parameter3 = 0;
  double yaw = 0;
  double angle = 0;
  const char *p = fileName.data();
//  std::cout <<"ININININININ" << std::endl;

  fs.open(p, std::ios::in);
  if (fs.fail()){
    std::cout <<"!!!!!!FATAL!!!!!!LOAD ROAD MAP FAIL!!!!!!" << std::endl;
    return;
   }else;
   while(getline(fs, line))
   {
    if (line.length() > 0){
      geometry_msgs::Pose roadPoint;
      std::stringstream ss(line);
      ss >> roadId >> roadPoint.position.x >> roadPoint.position.y >> angle >> roadType >> parameter2 >> parameter3;
      yaw = (angle*3.14)/180;
      roadPoint.orientation = tf::createQuaternionMsgFromYaw(yaw);

      path.poses.push_back(roadPoint);
      //std::cout << path << std::endl;

     }else;
   }
  fs.close();
 }

  void CubicSplineInterpolator::interpolatePath(
    const geometry_msgs::PoseArray& path,
    geometry_msgs::PoseArray& smoothedPath)
  {

    smoothedPath.header = path.header;
    interpolatePath(path.poses, smoothedPath.poses);
  }


  void CubicSplineInterpolator::interpolatePath(
    const std::vector<geometry_msgs::Pose>& path,
    std::vector<geometry_msgs::Pose>& smoothedPath)
  {
    // clear new smoothed path vector in case it's not empty
    smoothedPath.clear();

    // set skipPoints_ to 0 if the path contains has too few points
    unsigned int oldSkipPoints = skipPoints_;
    skipPoints_ = 0;
    //std::min<int>(path.size() - 2, skipPoints_);

    // create cummulative distances vector
    std::vector<double> cummulativeDistances;
    calcCummulativeDistances(path, cummulativeDistances);

    // create temp pose
    geometry_msgs::Pose pose;
    //pose.header = path[0].header;

    unsigned int numPoints = pointsPerUnit_ * calcTotalDistance(path);

    smoothedPath.resize(numPoints);

    // interpolate points on the smoothed path using the points in the original path
    for (unsigned int i = 0; i < numPoints; i++)
    {
      double u = static_cast<double>(i) / (numPoints-1);
      interpolatePoint(path, cummulativeDistances, pose, u);

      if (isnan(pose.position.x) || isnan(pose.position.y))
        pose = smoothedPath[std::max(static_cast<int>(i)-1, 0)];
      smoothedPath[i] = pose;
    }

    // copy start and goal orientations to smoothed path
    smoothedPath.front().orientation = path.front().orientation;
    smoothedPath.back().orientation = path.back().orientation;

    // interpolate orientations of intermediate poses
    for (unsigned int i = 1; i < smoothedPath.size()-1; i++)
    {
      double dx = smoothedPath[i+1].position.x - smoothedPath[i].position.x;
      double dy = smoothedPath[i+1].position.y - smoothedPath[i].position.y;
      double th = atan2(dy, dx);
      smoothedPath[i].orientation = tf::createQuaternionMsgFromYaw(th);
    }

    // revert skipPoints to original value
    skipPoints_ = oldSkipPoints;
  }


  void CubicSplineInterpolator::interpolatePoint(
    const std::vector<geometry_msgs::Pose>& path,
    const std::vector<double>& cummulativeDistances,
    geometry_msgs::Pose& point,
    double pointCummDist)
  {
    unsigned int group = findGroup(cummulativeDistances, pointCummDist);
    // ROS_INFO("u: %f, idx: %u", pointCummDist, group);

    double a = calcAlphaCoeff(path, cummulativeDistances, group, pointCummDist);
    double b = calcBetaCoeff(path, cummulativeDistances, group, pointCummDist);
    double c = calcGammaCoeff(path, cummulativeDistances, group, pointCummDist);
    double d = calcDeltaCoeff(path, cummulativeDistances, group, pointCummDist);

    std::vector<double> grad, nextGrad;
    calcPointGradient(path, cummulativeDistances, group, grad);
    calcPointGradient(path, cummulativeDistances, group+1, nextGrad);

    point.position.x =
      + a * path[group*(skipPoints_+1)].position.x
      + b * path[(group+1)*(skipPoints_+1)].position.x
      + c * grad[0]
      + d * nextGrad[0];

    point.position.y =
      + a * path[group*(skipPoints_+1)].position.y
      + b * path[(group+1)*(skipPoints_+1)].position.y
      + c * grad[1]
      + d * nextGrad[1];
  }


  void CubicSplineInterpolator::calcCummulativeDistances(
    const std::vector<geometry_msgs::Pose> path,
    std::vector<double>& cummulativeDistances)
  {
    cummulativeDistances.clear();
    cummulativeDistances.push_back(0);

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
      cummulativeDistances.push_back(
        cummulativeDistances.back()
        + calcDistance(path, i) / calcTotalDistance(path));
  }


  double CubicSplineInterpolator::calcTotalDistance(
    const std::vector<geometry_msgs::Pose>& path)
  {
    double totalDist = 0;

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
      totalDist += calcDistance(path, i);

    return totalDist;
  }


  double CubicSplineInterpolator::calcDistance(
    const std::vector<geometry_msgs::Pose>& path,
    unsigned int idx)
  {
    if (idx <= 0 || idx >=path.size())
      return 0;

    double dist =
      hypot(
        path[idx].position.x - path[idx-skipPoints_-1].position.x,
        path[idx].position.y - path[idx-skipPoints_-1].position.y);

    return dist;
  }


  double CubicSplineInterpolator::calcAlphaCoeff(
    const std::vector<geometry_msgs::Pose> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double alpha =
      + 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
      - 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2)
      + 1;

    return alpha;
  }


  double CubicSplineInterpolator::calcBetaCoeff(
    const std::vector<geometry_msgs::Pose> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double beta =
      - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
      + 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2);

    return beta;
  }


  double CubicSplineInterpolator::calcGammaCoeff(
    const std::vector<geometry_msgs::Pose> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double gamma =
      (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
       - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
      * (cummulativeDistances[idx+1] - cummulativeDistances[idx])
      + input
      - cummulativeDistances[idx];

    return gamma;
  }


  double CubicSplineInterpolator::calcDeltaCoeff(
    const std::vector<geometry_msgs::Pose> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double delta =
      (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
       - pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
      * (cummulativeDistances[idx+1] - cummulativeDistances[idx]);

      return delta;
  }


  double CubicSplineInterpolator::calcRelativeDistance(
    const std::vector<double>& cummulativeDistances,
    const unsigned int idx,
    const double input)
  {
    double relDist =
      (input - cummulativeDistances[idx])
      / (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
    return relDist;
  }


  void CubicSplineInterpolator::calcPointGradient(
    const std::vector<geometry_msgs::Pose>& path,
    const std::vector<double>& cummulativeDistances,
    unsigned int idx,
    std::vector<double>& gradient)
  {
    double dx, dy, du;
    gradient.assign(2, 0);

    // use either pose.yaw or interpolation to find gradient of points
    if ((useEndConditions_ && (idx == 0 || idx == cummulativeDistances.size()-1))
      || useMiddleConditions_)
    {
      double th = tf::getYaw(path[idx*(skipPoints_+1)].orientation);
      int sign = (fabs(th) < M_PI / 2) ? 1 : -1;

      gradient[0] = sign * calcTotalDistance(path)
        * sqrt(1 + pow(tan(th),2)) / (1 + pow(tan(th), 2));
      gradient[1] = tan(th) * gradient[0];
    }
    else  // gradient interpolation using original points
    {
      if (idx == 0 || idx == cummulativeDistances.size()-1)
        return;

      dx = path[(idx)*(skipPoints_+1)].position.x - path[(idx-1)*(skipPoints_+1)].position.x;
      dy = path[(idx)*(skipPoints_+1)].position.y - path[(idx-1)*(skipPoints_+1)].position.y;
      du = cummulativeDistances[idx] - cummulativeDistances[idx-1];

      gradient[0] =  dx / du;
      gradient[1] =  dy / du;
    }
  }


  unsigned int CubicSplineInterpolator::findGroup(
    const std::vector<double>& cummulativeDistances,
    double pointCummDist)
  {
    unsigned int i;
    for (i = 0; i < cummulativeDistances.size()-1; i++)
    {
      if (pointCummDist <= cummulativeDistances[i+1])
        return i;
    }
    return i;
  }

}  // namespace path_smoothing

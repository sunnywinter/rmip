#include "rmip/cubic_spline_interpolator.h"
#include <tf/tf.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rmip");
  ros::NodeHandle nh("~");
  std::string mappath = ros::package::getPath("rmip");
  std::string roadMap_path = mappath+"/maps/roadMap.txt";

 while (ros::ok())
  {
  //ros::Publisher initialPosePub = nh.advertise<geometry_msgs::Pose>("initial_pose", 1, true);
  //ros::Publisher finalPosePub = nh.advertise<geometry_msgs::Pose>("final_pose", 1, true);
  ros::Publisher pathPub = nh.advertise<geometry_msgs::PoseArray>("initial_path", 1, true);
  ros::Publisher smoothedPathPub = nh.advertise<geometry_msgs::PoseArray>("smoothed_path", 1, true);

  int pointsPerUnit, skipPoints;
  bool useEndConditions, useMiddleConditions;

  nh.param<int>("points_per_unit", pointsPerUnit, 5);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, true);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, true);

  geometry_msgs::PoseArray path, smoothedPath;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();


  // create a cubic spline interpolator
  path_smoothing::CubicSplineInterpolator csi("lala");

  csi.loadRoadMap(roadMap_path, path);
  csi.interpolatePath(path, smoothedPath);

  pathPub.publish(path);
  smoothedPathPub.publish(smoothedPath);
  ros::Rate loop_rate(20);
  loop_rate.sleep();
  }

  return 0;
}

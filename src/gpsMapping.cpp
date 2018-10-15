// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <math.h>
#include <iostream>
#include <fstream>
#include <ostream>

#include <gps_mapping/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "gps_mapping/GPTRA_MSG.h"
#include "gps_mapping/Heading.h"


#include <string>

std::ofstream outFile_groundTruth;

const float scanPeriod = 0.1;

const int stackFrameNum = 1;
const int mapFrameNum = 5;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeNavSatFix = 0;
double timeGL8GPTRAMsg = 0;
double timeGL8HeadingMsg = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newNavSatFix = false;
bool newGL8GPTRAMsg = false;
bool newGL8HeadingMsg = false;


const int laserCloudWidth = 61;
const int laserCloudHeight = 41;
const int laserCloudDepth = 7;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudCenWidth = laserCloudWidth/2;
int laserCloudCenHeight = laserCloudHeight/2;
int laserCloudCenDepth = laserCloudDepth/2;

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];


//参数配置
static const double Ellipse_a = 6378137;
static const double Ellipse_b = 6356752.3142;
static const double PI = 3.14159265358;
double Ellipse_L0 = 120.4620;

//gps初始点位置

double OriginX = 529246.000;
double OriginY = 3496650.000;

//gps偏移值,主要是和其他系统的对齐

double OffsetX = -1.1;
double OffsetY = 1.03;

double startPoint_x=0;
double startPoint_y=0;
double startPoint_z=0;
double startYaw=0;

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>()); 
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurround4(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCubeCorner(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCubeSurf(new pcl::PointCloud<PointType>());
bool isLaserCloudCornerExisted[laserCloudNum]={false};
bool isLaserCloudSurfExisted[laserCloudNum]={false};


double coordinate[6]={0};

bool systemInitiated=false;


void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
  timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

  laserCloudCornerLast->clear();
  pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices);

  newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
  timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

  laserCloudSurfLast->clear();
  pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudSurfLast,*laserCloudSurfLast, indices);

  newLaserCloudSurfLast = true;
}



void navSatFixHandler(const sensor_msgs::NavSatFix::ConstPtr &fixIn)
{
  timeNavSatFix = fixIn->header.stamp.toSec();

  static const double Ellipse_n = (Ellipse_a - Ellipse_b) / (Ellipse_a + Ellipse_b);
  static const double Ellipse_e = sqrt(Ellipse_a * Ellipse_a - Ellipse_b * Ellipse_b) / Ellipse_a;
  static const double Ellipse_ee = sqrt(Ellipse_a * Ellipse_a - Ellipse_b * Ellipse_b) / Ellipse_b;
  static const double Ellipse_C0 = (Ellipse_a + Ellipse_b) * (1 + 0.25 * pow(Ellipse_n, 2) + 0.015625 * pow(Ellipse_n, 4)) * 0.5;
  static const double Ellipse_C1 = -1.5 * Ellipse_n + 0.5625 * pow(Ellipse_n, 3) - 0.09375 * pow(Ellipse_n, 5);
  static const double Ellipse_C2 = 0.9375 * pow(Ellipse_n, 2) - 0.46875 * pow(Ellipse_n, 4);
  static const double Ellipse_C3 = -35 / 48 * pow(Ellipse_n, 3) + 0.41015625 * pow(Ellipse_n, 5);
  static const double Ellipse_C4 = 0.615234375 * pow(Ellipse_n, 4);

  double Ellipse_lat = fixIn->latitude * PI / 180;
  double Ellipse_lon = (fixIn->longitude - Ellipse_L0) * PI / 180;
  double Ellipse_N = Ellipse_a / sqrt(1 - pow(Ellipse_e * sin(Ellipse_lat), 2));
  double Ellipse_t = tan(Ellipse_lat);
  double Ellipse_g = Ellipse_ee * cos(Ellipse_lat);
  double Ellipse_m = cos(Ellipse_lat) * Ellipse_lon;

  double Ellipse_X = Ellipse_C0 * (Ellipse_lat + Ellipse_C1 * sin(2 * Ellipse_lat) + Ellipse_C2 * sin(4 * Ellipse_lat) + Ellipse_C3 * sin(6 * Ellipse_lat) + Ellipse_C4 * sin(8 * Ellipse_lat));
  //--------------------------solution----------------------------------

  double tempy = Ellipse_X + 0.5 * Ellipse_N * Ellipse_t * pow(Ellipse_m, 2);
  tempy += 0.041666666666666666666666666667 * Ellipse_N * Ellipse_t * (5 - pow(Ellipse_t, 2) + 9 * pow(Ellipse_g, 2) + 4 * pow(Ellipse_g, 4)) * pow(Ellipse_m, 4);
  tempy += 0.0013888888888888888888888888889 * Ellipse_N * Ellipse_t * (61 - 58 * pow(Ellipse_t, 2) + pow(Ellipse_t, 4) + 270 * pow(Ellipse_g, 2) - 330 * pow(Ellipse_g, 2) * pow(Ellipse_t, 2)) * pow(Ellipse_m, 6);
  tempy += 0.0000248015873015873 * Ellipse_t * Ellipse_N * pow(Ellipse_m, 8) * (1385 - 3111 * pow(Ellipse_t, 2) + 543 * pow(Ellipse_t, 4) - pow(Ellipse_t, 6));

  double tempx = Ellipse_N * Ellipse_m + Ellipse_N * pow(Ellipse_m, 3) * (1 - pow(Ellipse_t, 2) + pow(Ellipse_g, 2)) * 0.16666666666666666666666666666666666667;
  tempx += Ellipse_N * (5 - 18 * pow(Ellipse_t, 2) + pow(Ellipse_t, 4) + 14 * pow(Ellipse_g, 2) - 58 * pow(Ellipse_g, 2) * pow(Ellipse_t, 2)) * pow(Ellipse_m, 5) * 0.008333333333333333333333333333;
  tempx += Ellipse_N * (61 - 479 * pow(Ellipse_t, 2) + 179 * pow(Ellipse_t, 4) - pow(Ellipse_t, 6)) * pow(Ellipse_m, 7) * 0.000198412698412698 + 500000;

  tempx = tempx * 100 - OriginX * 100; //cm
  tempy = tempy * 100 - OriginY * 100; //cm

  tempx += OffsetX * 100;
  tempy += OffsetY * 100;

  //coordinate[0]=fixIn->header.stamp;
  coordinate[0] = (tempx) / 100.0;
  coordinate[1] = (tempy) / 100.0;
  coordinate[2] = fixIn->altitude;

  newNavSatFix = true;
}

void gl8GPTRAMsgHandler(const gps_mapping::GPTRA_MSG::ConstPtr &gl8GPTRAMsg)
{
  timeGL8GPTRAMsg = gl8GPTRAMsg->header.stamp.toSec();

  coordinate[3]=gl8GPTRAMsg->heading;

  newGL8GPTRAMsg = true;
}

void gl8HeadingMsgHandler(const gps_mapping::Heading::ConstPtr &gl8HeadingMsg)
{
  timeGL8HeadingMsg = gl8HeadingMsg->header.stamp.toSec();

  coordinate[3]=gl8HeadingMsg->data;

  newGL8HeadingMsg = true;
}

// void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn)
// {
//   double roll, pitch, yaw;
//   tf::Quaternion orientation;
//   tf::quaternionMsgToTF(imuIn->orientation, orientation);
//   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

//   imuPointerLast = (imuPointerLast + 1) % imuQueLength;

//   imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
//   imuRoll[imuPointerLast] = roll;
//   imuPitch[imuPointerLast] = pitch;
// }

void keyboardControlHandler(const std_msgs::String::ConstPtr &controlInfoIn)
{

  pcl::PCDWriter pcdCubeWriter;

  for (int i = 0; i < laserCloudWidth; ++i)
    for (int j = 0; j < laserCloudHeight; ++j)
      for (int k = 0; k < laserCloudDepth; ++k)
      {
        int ind = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
        char tmp[4] = {0};
        sprintf(tmp, "%02d", i);
        std::string strI = tmp;
        sprintf(tmp, "%02d", j);
        std::string strJ = tmp;
        sprintf(tmp, "%02d", k);
        std::string strK = tmp;

        if (isLaserCloudCornerExisted[ind])
        {
          laserCloudCubeCorner->clear();
          *laserCloudCubeCorner = *laserCloudCornerArray[ind];
          pcdCubeWriter.write("/home/zhou/Desktop/mapCubeData/corner/" + strI + "_" + strJ + "_" + strK + ".pcd", *laserCloudCubeCorner);
        }

        if (isLaserCloudSurfExisted[ind])
        {
          laserCloudCubeSurf->clear();
          *laserCloudCubeSurf = *laserCloudSurfArray[ind];
          pcdCubeWriter.write("/home/zhou/Desktop/mapCubeData/surf/" + strI + "_" + strJ + "_" + strK + ".pcd", *laserCloudCubeSurf);
        }
      }

  std::cout<<"Writing finished."<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpsMapping");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2, laserCloudCornerLastHandler);

  ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2, laserCloudSurfLastHandler);

  ros::Subscriber subNavSatFix = nh.subscribe<sensor_msgs::NavSatFix>("/strong/fix", 5, navSatFixHandler);

  ros::Subscriber subGPTRAMsg = nh.subscribe<gps_mapping::GPTRA_MSG>("/strong/attitude", 5, gl8GPTRAMsgHandler);

  /////////////////////////////////////////
  // ros::Subscriber subNavSatFix = nh.subscribe<sensor_msgs::NavSatFix>("/gps_filtered", 5, navSatFixHandler);
  // ros::Subscriber subGPTRAMsg = nh.subscribe<gps_mapping::Heading>("/yaw_filtered", 5, gl8HeadingMsgHandler);
  /////////////////////////////////////////

  //ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2, laserCloudFullResHandler);

  //ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imuHandler);

  ros::Subscriber subControlInfo = nh.subscribe<std_msgs::String>("controlInfo", 1000, keyboardControlHandler);

  ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);

  // ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);

  //ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("aft_mapped_to_init", 5);
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform aftMappedTrans;
  aftMappedTrans.frame_id_ = "camera_init";
  aftMappedTrans.child_frame_id_ = "aft_mapped";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  PointType pointOri, pointSel, pointProj, coeff;

  // cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  // cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  // cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  // cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  // cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  // cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  // bool isDegenerate = false;
  // cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

  pcl::VoxelGrid<PointType> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

  


  outFile_groundTruth.open("/home/zhou/Desktop/test_groundtruth_gps.txt");

  for (int i = 0; i < laserCloudNum; i++)
  {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>()); // 在最后方输入。
    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
  }

  int frameCount = stackFrameNum - 1;
  int mapFrameCount = mapFrameNum - 1;
  ros::Rate rate(100);
  bool status = ros::ok();

  while (status)
  {
    ros::spinOnce();

    if (newLaserCloudCornerLast && newLaserCloudSurfLast  && newNavSatFix && newGL8GPTRAMsg &&
        fabs(timeLaserCloudCornerLast - timeNavSatFix) < 0.05 &&
        fabs(timeLaserCloudSurfLast - timeNavSatFix) < 0.05 &&
        fabs(timeGL8GPTRAMsg - timeNavSatFix) < 0.05)
    {
      newLaserCloudCornerLast = false;
      newLaserCloudSurfLast = false;
      newNavSatFix = false;
      newGL8GPTRAMsg = false;
      //newGL8HeadingMsg = false;

      if (!systemInitiated)
      {
        startPoint_x = coordinate[0];
        startPoint_y = coordinate[1];
        startPoint_z = coordinate[2];

        startYaw = coordinate[3];

        systemInitiated = true;
      }

      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

      transform(0, 0) = cos(coordinate[3]);
      transform(0, 1) = -sin(coordinate[3]);
      transform(1, 0) = sin(coordinate[3]);
      transform(1, 1) = cos(coordinate[3]);

      transform(0, 3) = coordinate[0]-startPoint_x;
      transform(1, 3) = coordinate[1]-startPoint_y;
      transform(2, 3) = coordinate[2]-startPoint_z;

      double yawNow=coordinate[3];




      pcl::transformPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, transform);
      pcl::transformPointCloud(*laserCloudSurfLast, *laserCloudSurfLast, transform);

      int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      for (int i = 0; i < laserCloudCornerLastNum; i++)
      {

        pointSel = laserCloudCornerLast->points[i];
        laserCloudCornerStack2->push_back(pointSel);
      }

      int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
      for (int i = 0; i < laserCloudSurfLastNum; i++)
      {

        pointSel = laserCloudSurfLast->points[i];
        laserCloudSurfStack2->push_back(pointSel);
      }

      int centerCubeI = int((transform(0, 3) + 25.0) / 50.0) + laserCloudCenWidth;
      int centerCubeJ = int((transform(1, 3) + 25.0) / 50.0) + laserCloudCenHeight;
      int centerCubeK = int((transform(2, 3) + 25.0) / 50.0) + laserCloudCenDepth;

      // Note: int(-0.1)=0;
      if (transform(0, 3) + 25.0 < 0)
        centerCubeI--;
      if (transform(1, 3) + 25.0 < 0)
        centerCubeJ--;
      if (transform(2, 3) + 25.0 < 0)
        centerCubeK--;

      //std::cout<<centerCubeI<<'\t'<<centerCubeJ<<'\t'<<centerCubeK<<'\t'<<transformTobeMapped[3]<<'\n';

      // 如果取到的子cube在整个大cube的边缘，则将点对应的cube的索引向中心方向挪动一个单位，这样做主要是截取边沿cube。
      while (centerCubeI < 3)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int i = laserCloudWidth - 1;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            bool tmpCornerExisted = isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            bool tmpSurfExisted = isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            for (; i >= 1; i--)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

              isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudCornerExisted[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudSurfExisted[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;
            isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpCornerExisted;
            isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpSurfExisted;

            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeI++;
        laserCloudCenWidth++;
      }

      while (centerCubeI >= laserCloudWidth - 3)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int i = 0;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            bool tmpCornerExisted = isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            bool tmpSurfExisted = isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            for (; i < laserCloudWidth - 1; i++)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

              isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudCornerExisted[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudSurfExisted[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;
            isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpCornerExisted;
            isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpSurfExisted;

            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeI--;
        laserCloudCenWidth--;
      }

      while (centerCubeJ < 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int j = laserCloudHeight - 1;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            bool tmpCornerExisted = isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            bool tmpSurfExisted = isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            for (; j >= 1; j--)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];

              isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudCornerExisted[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
              isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudSurfExisted[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;

            isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpCornerExisted;
            isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpSurfExisted;

            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
      }

      while (centerCubeJ >= laserCloudHeight - 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int j = 0;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            bool tmpCornerExisted = isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            bool tmpSurfExisted = isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            for (; j < laserCloudHeight - 1; j++)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];

              isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudCornerExisted[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
              isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudSurfExisted[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;

            isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpCornerExisted;
            isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpSurfExisted;

            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
      }

      while (centerCubeK < 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int j = 0; j < laserCloudHeight; j++)
          {
            int k = laserCloudDepth - 1;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            bool tmpCornerExisted = isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            bool tmpSurfExisted = isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            for (; k >= 1; k--)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];

              isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
              isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;

            isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpCornerExisted;
            isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpSurfExisted;

            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeK++;
        laserCloudCenDepth++;
      }

      while (centerCubeK >= laserCloudDepth - 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int j = 0; j < laserCloudHeight; j++)
          {
            int k = 0;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            bool tmpCornerExisted = isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            bool tmpSurfExisted = isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];

            for (; k < laserCloudDepth - 1; k++)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];

              isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
              isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                  isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;

            isLaserCloudCornerExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpCornerExisted;
            isLaserCloudSurfExisted[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = tmpSurfExisted;

            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeK--;
        laserCloudCenDepth--;
      }

      int laserCloudSurroundNum = 0;
      for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
      {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
        {
          for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
          {
            if (i >= 0 && i < laserCloudWidth &&
                j >= 0 && j < laserCloudHeight &&
                k >= 0 && k < laserCloudDepth)
            {
              laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
              laserCloudSurroundNum++;
            }
          }
        }
      }



        ////////////////////////////////////////////////////////////
        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        laserCloudCornerStack2->clear();
        laserCloudSurfStack2->clear();
        ////////////////////////////////////////////////////////////



        //==============================================================//
        //laserCloudCornerStack是当前帧的坐标，pointAssociateToMap函数后的pointSel坐标是全局地图坐标。
        for (int i = 0; i < laserCloudCornerStackNum; i++)
        {
          pointSel = laserCloudCornerStack->points[i];

          int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

          if (pointSel.x + 25.0 < 0)
            cubeI--;
          if (pointSel.y + 25.0 < 0)
            cubeJ--;
          if (pointSel.z + 25.0 < 0)
            cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth &&
              cubeJ >= 0 && cubeJ < laserCloudHeight &&
              cubeK >= 0 && cubeK < laserCloudDepth)
          {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudCornerArray[cubeInd]->push_back(pointSel);
            isLaserCloudCornerExisted[cubeInd] = true;
          }
        }

        for (int i = 0; i < laserCloudSurfStackNum; i++)
        {
          pointSel=laserCloudSurfStack->points[i];

          int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

          if (pointSel.x + 25.0 < 0)
            cubeI--;
          if (pointSel.y + 25.0 < 0)
            cubeJ--;
          if (pointSel.z + 25.0 < 0)
            cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth &&
              cubeJ >= 0 && cubeJ < laserCloudHeight &&
              cubeK >= 0 && cubeK < laserCloudDepth)
          {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudSurfArray[cubeInd]->push_back(pointSel);
            isLaserCloudSurfExisted[cubeInd]=true;
          }
        }
        //==============================================================//

        // (降采样滤波????????)
        for (int i = 0; i < laserCloudSurroundNum; i++)
        {
          int ind = laserCloudSurroundInd[i];

          laserCloudCornerArray2[ind]->clear();
          downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
          downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

          laserCloudSurfArray2[ind]->clear();
          downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
          downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

          pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
          laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
          laserCloudCornerArray2[ind] = laserCloudTemp;

          laserCloudTemp = laserCloudSurfArray[ind];
          laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
          laserCloudSurfArray2[ind] = laserCloudTemp;
        }


        // 发布频率为 5Hz。
        mapFrameCount++;
        if (mapFrameCount >= mapFrameNum)
        {
          mapFrameCount = 0;

          laserCloudSurround2->clear();

          for (int i = 0; i < laserCloudSurroundNum; i++)
          {
            int ind = laserCloudSurroundInd[i];
            *laserCloudSurround2 += *laserCloudCornerArray[ind];
            *laserCloudSurround2 += *laserCloudSurfArray[ind];

          }
          
          laserCloudSurround->clear();
          downSizeFilterCorner.setInputCloud(laserCloudSurround2);
          downSizeFilterCorner.filter(*laserCloudSurround);

          sensor_msgs::PointCloud2 laserCloudSurround3;
          pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
          laserCloudSurround3.header.stamp = ros::Time().fromSec(timeNavSatFix);
          laserCloudSurround3.header.frame_id = "camera_init";
          pubLaserCloudSurround.publish(laserCloudSurround3);

        }



        outFile_groundTruth << std::fixed << std::setprecision(12) << timeNavSatFix << '\t' << transform(0, 3) << '\t' << transform(1, 3)<< '\t' << transform(2, 3)<< '\t' << yawNow<< std::endl;


        // aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        // aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        // aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3],
        //                                      transformAftMapped[4], transformAftMapped[5]));
        // tfBroadcaster.sendTransform(aftMappedTrans);

        ///Write data to file.

        // outfile << odomAftMapped.pose.pose.position.x << '\t' << odomAftMapped.pose.pose.position.y << '\t' << odomAftMapped.pose.pose.position.z << '\n';

      
    }

    status = ros::ok();
    rate.sleep();
  }

  outFile_groundTruth.close();

  std::cout<<laserCloudCenWidth<<'\t'<<laserCloudCenHeight<<'\t'<<laserCloudCenDepth<<'\n';

  return 0;
}

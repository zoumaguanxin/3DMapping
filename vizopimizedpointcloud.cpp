#include <iostream>
#include "fileread.hpp"
#include "viz.hpp"
#include "pointCloudDeal.hpp"

using namespace std;

int main(int argc, char **argv) {
   
  char dir[100]="/home/shaoan/projects/SLAM6D/dat_et4/";
  char posedir[100]="/home/shaoan/projects/pose_estimate_3d/data/";
  const int maxnumFile=15;
  int strat_num=0;
  vector<map3d::pose> pose_optimized;  
   pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
   map3d::readg2ofile(posedir,pose_optimized);
   //define a pointor container used to store all PointCloud

  for(int numFile=0;numFile<maxnumFile-strat_num;numFile++)
  {    
     int readnum=strat_num+numFile;

     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
     map3d::readPointCloud(dir,readnum,current_cloud_ptr);
     Eigen::Matrix3f R_current=pose_optimized[readnum].q.matrix();
     Eigen::Vector3f t_current=pose_optimized[readnum].t; 
    map3d::transform(current_cloud_ptr,global_cloud_ptr,t_current,R_current);    
  }
  
  //***********************************************************
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
       viewer =map3d::simpleVis(global_cloud_ptr);
	while (!viewer->wasStopped ())
	  {
		  if(!viewer->wasStopped ())          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
   
    return 0;
}

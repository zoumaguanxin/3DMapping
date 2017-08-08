#include <iostream>
#include "fileread.hpp"
#include "pairwiseICP.hpp"
#include "viz.hpp"
#include "pointCloudDeal.hpp"

using namespace std;

#define PREDICT_ENABLE true
#define ICP_ENALBE true

int main(int argc, char **argv) {
   
  char dir[100]="/home/shaoan/projects/SLAM6D/dat_et4/";
  /**
   * \a maxnumFile is the  number of reading files.
   */
  const int maxnumFile=15;
  int strat_num=0;
  
   pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
   
   //define a pointor container used to store all PointCloud
   Eigen::Matrix3f R_initial;
   Eigen::Vector3f t_initial; 
  for(int numFile=0;numFile<maxnumFile;numFile++)
  {    
     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
     /**
      * \a R_odometry store the rotation part of odometry data. 
      * \b R_initial_current represent that transform the current frame to initial frame. means that R_current=R_initial*R_initial_current.
      * \c R_current is represented in global frame, that can transform point cloud into global frame. The complete label should be R_global_current. 
      * But for simplication, we don't include "global" label. All labels that has only one sublabel represent R_global_SUBLABEL.      * 
      */
     Eigen::Matrix3f R_odometry, R_initial_current,R_current,R_last_odom;
       /**
      * \a t_odometry store the translation part of odometry data. 
      * \b t_initial_current represent that transform the current frame to initial frame. means that R_current=R_initial*R_initial_current.
      * \c t_current is represented in global frame, that can transform point cloud into global frame. The complete label should be R_global_current. 
      * But for simplication, we don't include "global" label. All labels that has only one sublabel represent R_global_SUBLABEL. 
      * It is the translation from "global" frame to "sublabel" frame. It also is represented as origin of source frame in target frame.
      */
     Eigen::Vector3f t_odometry, t_initial_current, t_current, t_last_odom;   
     int readnum=strat_num+numFile;
     map3d::readfile(dir,readnum,current_cloud_ptr,R_odometry,t_odometry);
     
     if(ICP_ENALBE)
	  {
	  if(numFile==0)
	  {
	    //global_cloud_ptr=current_cloud_ptr;
	    R_initial=R_odometry;
	    t_initial=t_odometry;
	    R_last_odom=R_odometry;
	    t_last_odom=t_odometry;
	    map3d::transform(current_cloud_ptr,global_cloud_ptr,t_initial,R_initial);     
	  }
	  else
	  {
	    //*************pairwiseICP*******************************
	    map3d::ICP icp(1000,1010,10,25);
	    icp.setInputCloud(current_cloud_ptr,global_cloud_ptr);
	    icp.setParamsConvergence();
	    
	    //判断是否启用预测，如果有，则用之前配准的位置纠正上一个时刻里成绩数据，并整合控制命令，否则直接使用里成绩数据。
	    if(PREDICT_ENABLE&&(numFile>1))
	    {
	      /**
	       * \a delta_R the odometry record the rotation between two closest scans, current->last
	       * \a delta_t the odometry record the translation between two closest scans, current->last
	       */
	    Eigen::Matrix3f delta_R;
	    Eigen::Vector3f delta_t;
	    //R10*R02=R12
	    delta_R=R_last_odom.transpose()*R_odometry;
	    delta_t=R_last_odom.transpose()*(t_odometry-t_last_odom);
	    //R01*R12=R02
	    R_current=R_current*delta_R;
	    t_current=t_current+R_current*delta_t;
	    R_last_odom=R_odometry;
	    t_last_odom=t_odometry;
	    }
	    else{
	    R_current=R_odometry;
	    t_current=t_odometry;
	    }
	    icp.solve(R_current,t_current);
	    if(icp.flag==false)
	    {
	      cout<<readnum<<"failed to register ";
	    }
	    cout<<R_current<<endl;
	    Eigen::Quaternion<float> q_current(R_current);       
	    //show the Quaternion style
	    cout<<q_current.coeffs()<<endl;       
	    // transform Quaternion to rotation matrix, also can use member function toRotationMatrix()
	    cout<<q_current.matrix();
	    //cout<<q_current.toRotationMatrix();
	    map3d::transform(current_cloud_ptr,global_cloud_ptr,t_current,R_current);     
	  }
     }
     else
     {
       map3d::transform(current_cloud_ptr,global_cloud_ptr,t_odometry,R_odometry);  
    }
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

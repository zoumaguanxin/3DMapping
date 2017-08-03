/**
 * \author Shaoan Zhao
 * \brief In this file, we will use two values to determine whether the loop is detected. One is the odometry pose. 
 * One value is the Euclidean distance between odometry poses, just refer to translation. If robot pose close to other place it ever traveled
 * Another is whether ICP can find reasonable registration. Although we believe that Mahalanobis distance between odometry poses usually is enough capable of detecting loop.
*/

#include<Eigen/Geometry>
#include<Eigen/Dense>
#include<pcl/common/common_headers.h>
#include<pcl/point_cloud.h>

#include<ctime>
#include<iostream>

using namespace std;

namespace map3d{
namespace backend{
    
    
    /**
     * \brief loop is a class that used to find a loop. The basic strategy to find a loop is to judge whether distance of two lcation is small 
     * \memberof distance_threshold 
     * \memberof flag , if flag is true, loop has been found
     *      * location also can be represented by pointXYZ. so can set location as a kdtree
     */
    class Loop
    {
    public:
      /**
       * \memberof   Loop
       */
      float distance_threshold;
      /**
       * \memberof flag 
       */
      bool flag;
      
      /**
       * \brief construct function
       */
      Loop(float distance_threshold_, bool flag_=false)
      {
	distance_threshold=distance_threshold_;
	flag=flag_;
      }
      
      /** 
       * \brief   detect loop from history data
       * \param[in] historyTrans 
       * \param[in] currentTrans
       * \return true if a loop has been detected
       */
     bool flagLoop(const vector<Eigen::Vector3f> &historyTrans, const Eigen::Vector3f &currentTrans)
      {
	int i=0;
	for(vector<Eigen::Vector3f>::const_iterator it= historyTrans.begin();it!=historyTrans.end();it++)
	  {
	    float distance;
	    Eigen::Vector3f residual;
	    residual=*it-currentTrans;
	    distance=residual.dot(residual);
	    distance=sqrt(distance);
	      if(distance<distance_threshold)
	      {
		flag=true;
		index.push_back(i);
	      }
	      i++;
	     }
	     return flag;
      }
      
      /**
       * \brief To get indexes of point cloud where loop has been detected 
       * \param[out] index_ is a vector container used to get the index of point cloud where a loop has been found.
       */
     void getLoopIndex(vector<int> &index_)
      {
	if(flag==true)
	{
	  index_=index;
	}
	else
	{
	  cout<<"error request for a loop, beacuse no loop has been detected"<<endl;
	}
      }
    private:
      vector<int> index;      
    };
 
    
  }   
}
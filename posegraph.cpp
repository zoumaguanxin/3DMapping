#include "fileread.hpp"
#include "pairwiseICP.hpp"
#include "pointCloudDeal.hpp"
#include "viz.hpp"
#include "loopdetect.hpp"
//#include "types.hpp"
#include <boost/graph/graph_concepts.hpp>
#include<Eigen/Dense>

using namespace std;

ostream& operator<<(ostream& out, const Eigen::Quaternion<float>& s)
{
  out<<s.x()<<" "<<s.y()<<" "<<s.z()<<" "<<s.w();
  return out;
}

ostream& operator<<(ostream& out, const Eigen::Vector3f & s)
{
  for(int i=0;i<s.rows();i++)
  {
    if(i<s.rows()-1)
    {
    out<<s(i)<<" ";
    }
    else{
     out<<s(i);
    }
  }
  return out;
}


int main(int argc, char**argv)
{
  string dir="/home/shaoan/projects/SLAM6D/dat_et4/";
  const int maxnumFile=40;
  vector<Eigen::Vector3f> historyTrans;
  vector<Eigen::Matrix3f> historyRotations;

  Eigen::Matrix3f R_current;
  Eigen::Vector3f t_current;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> historyPointCloudPtr;
  vector<map3d::vertix<map3d::pose> > initialVertix;
  vector<map3d::edge<map3d::pose> >  edges;
  for(int nfile=0;nfile<maxnumFile;nfile++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    map3d::readfile(dir.c_str(),nfile,currentPointCloud,R_current,t_current);
    
    //store the VERTEX_SE3
    map3d::pose tempose;
    tempose.q=R_current;
    tempose.t=t_current;
    map3d::vertix<map3d::pose>  temVertix;
    temVertix.Header="VERTEX_SE3:QUAT";
    temVertix.pose_ID=nfile;
    temVertix.initialPose=tempose;
    initialVertix.push_back(temVertix);
    //store the indexes of PointCloud
    historyPointCloudPtr.push_back(currentPointCloud);
    
    //if number of file bigger than 1, examine whether a edge can be detected from history trace. 
    vector<int> indexes;
     map3d::backend::Loop loop(500);  
    if(nfile>0)
    {
      if(loop.flagLoop(historyTrans,t_current))
      {
	cout<<"label 1";
	loop.getLoopIndex(indexes);
      for(int i=0;i<indexes.size();i++)
      {
	map3d::edge<map3d::pose> temedge;
	Eigen::Matrix3f R_delta=R_current.transpose()*historyRotations[indexes[i]];
	Eigen::Vector3f t_delta=R_current.transpose()*(historyTrans[indexes[i]]-t_current);
	//cout<<" label 2";
	map3d::ICP pairwiseicp(500,1010,10,25);
	pairwiseicp.setInputCloud(historyPointCloudPtr[indexes[i]],currentPointCloud);
        pairwiseicp.setParamsConvergence();
	pairwiseicp.solve(R_delta,t_delta);
	//cout<<" label 3";
      //If a edge is added, we need to compute related InformationMatrix and store it.
        if(pairwiseicp.flag==true)
	{
	  map3d::pose tempose;
	  tempose.q=R_delta;
	  tempose.t=t_delta;
	  Eigen::Matrix<float,6,6> InformationMatrix;
	  //cout<<"label 5";
	  pairwiseicp.getInformationMatrix(InformationMatrix);
	 // cout<<" label 4";
	  temedge.a_ID=nfile;
	  temedge.b_ID=indexes[i];
	  temedge.Header="EDGE_SE3:QUAT";
	  temedge.pose_ab=tempose;
	  temedge.informationMatrix_ab=InformationMatrix;
	  edges.push_back(temedge);
	}
       }
      }
    }    
     historyTrans.push_back(t_current); 
     historyRotations.push_back(R_current);
  }
  
  
     ofstream file;
     //ios_base::out is same as ios_base::trunc.Open the file for output. if the file existed, it will be discarded.
     file.open("indoor.g2o",ios_base::out);
     if(file.good())
     {
       for(int i=0;i<initialVertix.size();i++)
       {
	 file<<initialVertix[i].Header<<" "<<initialVertix[i].pose_ID<<" "<< initialVertix[i].initialPose.t<<" "<<initialVertix[i].initialPose.q<<endl;
	}
	for(int i=0;i<edges.size();i++)
	{
	  file<<edges[i].Header<<" "<<edges[i].a_ID<<" "<<edges[i].b_ID;
	  file<<" "<<edges[i].pose_ab.t<<" "<<edges[i].pose_ab.q<<" ";
	  for(int k=0;k<6;k++)
	  {
	    for(int l=k;l<6;l++)
	    {
	      file<<edges[i].informationMatrix_ab(k,l)<<" ";
	    }
	  }
	  file<<endl;
	}
    }
    

}
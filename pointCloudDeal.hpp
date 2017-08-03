#include<pcl/common/common_headers.h>
 #include<pcl/point_cloud.h>
#include<Eigen/Dense>
 
 namespace map3d{
   
   void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcp, pcl::PointCloud<pcl::PointXYZ>::Ptr dst_pcp,const Eigen::Vector3f &tranlsation, const Eigen::Matrix3f &Rotation)
   { 
     for(int j=0;j<src_pcp->points.size();++j)
	{
	  Eigen::Vector3f temV;
	  temV<<src_pcp->points[j].x,src_pcp->points[j].y,src_pcp->points[j].z;
	  temV=Rotation*temV;
	  pcl::PointXYZ temPoint;
	  temPoint.x=temV(0)+tranlsation(0);
	  temPoint.y=temV(1)+tranlsation(1);
	  temPoint.z=temV(2)+tranlsation(2);
	  dst_pcp->points.push_back(temPoint);
	}
  }
  
  void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcp, pcl::PointCloud<pcl::PointXYZ>::Ptr dst_pcp,const Eigen::Vector3f &translation, const Eigen::Quaternion<float> &Rotation)
  {
      Eigen::Matrix3f RotationM=Rotation.toRotationMatrix();
      transform(src_pcp,dst_pcp,translation,RotationM);
  }
}
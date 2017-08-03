  #include<pcl/common/file_io.h>
  #include<pcl/point_cloud.h>
  #include<vector>
  #include<ctime>
  #include<fstream>
  #include<iostream>
 #include<utility>
  
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

namespace map3d{
  /**
   * \brief  vanilla-R iterative closest point algorithm
   * \example
   * map3d::ICP icp(k,search_r_up,d_threshold,step_degrade);
   * icp.setInputCloud(src,dst);
   * icp.setParamsConvergence();
   * icp.solve(R,t) 
   */
  class ICP{
    public:
      string Options;
      /**
       * \member k , algorithm ask number of the eligable pairs to be up to k, or the algorithm will be unstable, and then the flag wiil be set as flase 
       */
      int k;
      /**
       * \memberof search_r_low  normally is zero, don't change it
       */
      float search_r_low;
      float search_r_up;
      //Because search raduis must be a positive value, so the floor_r set a min upper bound     
      float floor_r;
      float step_degrade;      
      float d_threshold;
       //In this programm, a search time for matching points in different point cloud is used to determine whether ICP can continue to run, if search timeout, flag is false
      bool flag;
      float timeout;      
      //the very closest point to origin usually is noise point, so the bounds is used to filter those nosie points.
      float filter_low_bounds;
      /**
     * \member final_pairs store the final index of point pairs used to solve transform. That are useful for compute InformationMatrix for pose graph 
     */
	vector< pair<int, int> > final_pairs;
      
      /**
       * @brief ICP constructor function, there are huge amounts of parameters need to be set.
       * All parameters has a default value.  normally the four parameters in left  are especially important for user to adpat their situations.
       */
      ICP(int k_=500,float search_r_up_=1010, float d_threshold_=10,float step_degrade_=25,float floor_r_=5,float search_r_low_=0, float filter_low_bounds_=40, float timeout_=0.5, bool flag_=true)
      {
	k=k_;
	search_r_low=search_r_low_;
	search_r_up=search_r_up_;
	d_threshold=d_threshold_;
	step_degrade=step_degrade_;	
	floor_r=floor_r_;
	flag=flag_;
	filter_low_bounds=filter_low_bounds_;
	timeout=timeout_;
      }
    
      /**
       * @brief setInputCloud for registering. 
       * @param SCP is source point cloud
       * @param TCP is target point cloud
       */
      void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr SCP,   pcl::PointCloud<pcl::PointXYZ>::Ptr TCP)
      {
	sourceCloudPtr=SCP;
	targetCloudPtr=TCP;
      }
      
      
      /** 
       *\brief 
       *\param[in] iter_maxnum
       *\param[out] 
       */ 
       void setParamsConvergence( int iter_maxnum_=80, int convergence_counter_max_=2, float sigma_=2.1,float error_belta_=0.2)
     {
       iter_maxnum=iter_maxnum_;
       convergence_counter_max=convergence_counter_max_;
       error_sigma=sigma_;
       error_belta=error_belta_;
    }
      
      //**************************************************************************************************
      bool Vanllia_R(int index, int &pairIndex, pcl::PointCloud<pcl::PointXYZ>::ConstPtr middle_cloud_ptr,const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree ,  const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree1) const
      {	
	  size_t K=1;
	  vector<int> pointIdxNKNSearch(K);//用来存放搜索到的点的index
	  vector<int> pointIdxNKNSearch1(K);
	  vector<float> pointNKNSquaredDistance(K);//存放搜索到的点到当前的欧式距离
	  vector<float> pointNKNSquaredDistance1(K);
	  kdtree.nearestKSearch (middle_cloud_ptr->points[index], K, pointIdxNKNSearch, pointNKNSquaredDistance);
	  pairIndex=pointIdxNKNSearch[0];
	  //ICRP
	  if((pointNKNSquaredDistance[0]<=search_r_up)&&(pointNKNSquaredDistance[0]>=search_r_low))
	  {
		kdtree1.nearestKSearch(targetCloudPtr->points[pointIdxNKNSearch[0]],K,pointIdxNKNSearch1, pointNKNSquaredDistance1);
		double d;
		d=pow(abs(middle_cloud_ptr->points[pointIdxNKNSearch1[0]].x-middle_cloud_ptr->points[index].x),2)
		  +pow(abs(middle_cloud_ptr->points[pointIdxNKNSearch1[0]].y-middle_cloud_ptr->points[index].y),2)
		  +pow(abs(middle_cloud_ptr->points[pointIdxNKNSearch1[0]].z-middle_cloud_ptr->points[index].z),2);  
		d=sqrt(d);
		if(d<d_threshold)
		{
		    return true;
		}
		else
		{
		    return false;
		}
	    }
	  else
	  {
	    return false;
	  }
      }
      
      
      /**
       *\brief SVD decompostion is used to solve the transform between two point cloud while the pair relation is knowned. This function can be improved using index of pairs instead of pairs themselves
       * \param[in]
       * \param[in]
       * \param[out]
       * \param[out]
       */
    void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr filter_initPoints,pcl::PointCloud<pcl::PointXYZ>::Ptr filter_matchedPoints,Eigen::Matrix3f &R_svd,Eigen::Vector3f &t_translate)
    {
        Eigen::Vector3f centriod_initPoints(0,0,0);//初始点云中心,当向量维数小于4的时候，这样的初始化也是被允许的
	Eigen::Vector3f centriod_matchedPoints(0,0,0);//匹配到的点云中心
	Eigen::Matrix3f U,V,S;	
	size_t filter_k;//经过ICRP过滤后的点对数目
	filter_k=filter_initPoints->points.size();
	Eigen::MatrixXf W;
	for(int i=0;i<filter_k;i++)
	{   Eigen::Vector3f temV;
	    temV<<filter_initPoints->points[i].x,filter_initPoints->points[i].y,filter_initPoints->points[i].z;//p
		centriod_initPoints+=temV;
		temV<<filter_matchedPoints->points[i].x,filter_matchedPoints->points[i].y,filter_matchedPoints->points[i].z;//y
		centriod_matchedPoints+=temV;
	}
	centriod_initPoints/=(float)filter_k;
	centriod_matchedPoints/=(float)filter_k;
	for(int i=0;i<filter_k;i++)
	{
		 Eigen::Vector3f temV1;
		 Eigen::Vector3f temV2;
		 temV1<<filter_initPoints->points[i].x,filter_initPoints->points[i].y,filter_initPoints->points[i].z;
		 temV1-=centriod_initPoints;
		 temV2<<filter_matchedPoints->points[i].x,filter_matchedPoints->points[i].y,filter_matchedPoints->points[i].z;
		 temV2-=centriod_matchedPoints;
		 if (i==0) {W=temV2*temV1.transpose();}
		 else {W+=temV2*temV1.transpose();}//y*p'
	}
	W/=(float)filter_k;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);	
	U=svd.matrixU();//不建议写成Eigen::Matrix3f U=svd.matrixU();
	V=svd.matrixV();
	S<<1,0,0,
	   0,1,0,
	   0,0,U.determinant()*V.determinant();
	R_svd=U*S*V.transpose();
	//y-R*p.求出的是initPoints点所在坐标系到matchedPoints点所在坐标系位移,注意这个位移是表示在matchedPoints坐标系下的,这里也就是global坐标系下
	t_translate=centriod_matchedPoints-R_svd*centriod_initPoints;
	cout<<"find the rotation Matrix using SVD method:"<<endl
		<<R_svd<<endl
		<<"平移："<<endl
		<<t_translate<<endl;	
    }
    
    
    
      /** 
       *@brief set a initial pose esimation, and then this function will solve a optimazition for registration. 
       * If icp failed to match points, member flag will be set as false, and this funtion will return initial estimation as final result. \brief
       * 
       *\param[in] R_0
       *\param[in] t_0
       * \param[out] R_k
       * \param[out] t_k
       */
      void solve(Eigen::Matrix3f &R_k, Eigen::Vector3f &t_k)
      {	
	R_update=R_k;//不应该再转置了，数据中的意思就是第二次到第一次的位姿
	t_update=t_k;
	pcl::PointXYZ initPoints[k];//随机找出k个点云并存放在这个类数组中
	pcl::PointXYZ points_transformed;//并应用初始旋转
	pcl::PointXYZ matchedPoints[k];//匹配得到的点
	pcl::PointXYZ matchedPoints1[k];
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	
	kdtree.setInputCloud(targetCloudPtr);//把点云按照kdtree形式存储
	float error_current;
	float error_last=0;
	int convergence_counter=0;
	int iterative_counter=0;
	while(convergence_counter<=convergence_counter_max&&iterative_counter<iter_maxnum)
	{
	    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_initPoints(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_matchedPoints(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	    //store index of pairs. for example (j, k) represent that jth point in source point cloud  is assicated with kth point in Targetcloud
	    vector<pair<int, int> > index_pairs;
	    Eigen::Vector3f v1;
	    Eigen::Vector3f v2;

	    for(int i=0;i<sourceCloudPtr->points.size();i++)
	    {
		v1(0)=sourceCloudPtr->points[i].x;
		v1(1)=sourceCloudPtr->points[i].y;
		v1(2)=sourceCloudPtr->points[i].z;
		v2=R_update*v1;//应用初始的旋转
		points_transformed.x=v2(0)+t_update(0);
		points_transformed.y=v2(1)+t_update(1);
		points_transformed.z=v2(2)+t_update(2);
		middle_cloud_ptr->points.push_back(points_transformed);
	    }
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_midCloudPtr;
	kdtree_midCloudPtr.setInputCloud(middle_cloud_ptr);
	int nPoint_current=sourceCloudPtr->points.size();
	int *p=new int;
	int i=0;
	int counter=0;
	time_t begin_search=clock();
	while(i<k)
	{    
		int j;
		unsigned int R;
		unsigned int t;
		time_t search_t=clock();
		if(float(search_t-begin_search)/CLOCKS_PER_SEC>timeout)
		{
		  flag=false;
		  break;
		}
L1:		 R = RAND_MAX-(RAND_MAX+1)%nPoint_current; //去除尾数
                 t = rand();
                while( t > R ) t = rand();
                j = t % nPoint_current;
		if (counter==0) p[counter]=j;
		else
		{
		  for(int m=0;m<counter;m++)
		  {
		    if(p[m]==j)
		    {
		      cout<<"back";
		      goto L1;      
		    }   
		  }
		 p[++counter]=j;
		 }
		float filter_r;//激光近距离点是用来标记的，不用于匹配	   
		filter_r=sqrt(pow(sourceCloudPtr->points[j].x,2)+pow(sourceCloudPtr->points[j].y,2)+pow(sourceCloudPtr->points[j].y,2));
		if(filter_r>filter_low_bounds)
		{    
		  int pairIndex;
		  bool flag_vanllia_R=Vanllia_R(j,pairIndex,middle_cloud_ptr,kdtree,kdtree_midCloudPtr);
		  if(flag_vanllia_R)
		  {
		    filter_initPoints->points.push_back(sourceCloudPtr->points[j]);
		    filter_matchedPoints->points.push_back(targetCloudPtr->points[pairIndex]);
		    pair<int,int> tempair(j,pairIndex);
		    index_pairs.push_back(tempair);    
		    i++;
		  }
		}	
	}
	
	if(flag==true)
	{
	  final_pairs=index_pairs;
	   // final_pairs.assign(index_pairs.begin(),index_pairs.end());
	    registration(filter_initPoints,filter_matchedPoints,R_update,t_update);
	    
	    //******************************************************************************************
	    //用于配准的内部点对误差统计
	    //******************************************************************************************
	    error_current=0;	
	    for(int j=0;j<filter_initPoints->size();++j)
	    {
		    Eigen::Vector3f temV1;
		    Eigen::Vector3f temV2;
		    temV1<<filter_initPoints->points[j].x,filter_initPoints->points[j].y,filter_initPoints->points[j].z;
		    temV2<<filter_matchedPoints->points[j].x,filter_matchedPoints->points[j].y,filter_matchedPoints->points[j].z;
		    temV1=R_update*temV1+t_update-temV2;
		    error_current+=sqrt(temV1.dot(temV1));
	    }
	    //note that error_final should satisfy following equation rather than "error_final=error_current"
	    error_final=pow(error_current,2);
	    /*******************************************************************************************/
	    
	    
	    //*******************************************************************************************
	    //收敛准则
	    //*******************************************************************************************
	    int filter_num=filter_initPoints->points.size();
	    if(abs(error_current-error_last)/filter_num<error_belta&&(error_current/(float)filter_num<error_sigma)) convergence_counter+=1;
	    error_last=error_current;
	    if(search_r_up>(step_degrade+floor_r)) search_r_up-=step_degrade;
	    else { search_r_up=floor_r;}
	    cout<<"the cureent error is equal to:"<<error_current<<endl;
	    iterative_counter++;
	    cout<<"the number of iterative:"<<iterative_counter<<endl;
	    }
	else
	{
	      break;
	 }
	}
	R_k=R_update;
	t_k=t_update;
      }
      
      /**
       * \brief compute infromation matrix for pose graph. Of course, it is not necessary for the situation where registering only two point cloud 
       * InformationMatrix was detemined by registration error and registration residual. The smaller current registration error, the more important the constraints.
       *one term of registeration residual represents the influence of individual variables on registeration. The bigger it, the more important the  variable.
       * 
       */
      void getInformationMatrix(Eigen::Matrix<float,6,6> &InformationMatrix)
      {
	InformationMatrix.setZero();
	if(flag==true)
	{
	 // cout<<final_pairs.size();
	  for(int i=0;i<final_pairs.size();i++)
	  {
	    Eigen::Vector3f temV1;
	    Eigen::Vector3f temV2;
	    /**
	     * 
	    temV1<<sourceCloudPtr->points[final_pairs[i].first].x,sourceCloudPtr->points[final_pairs[i].first].y,sourceCloudPtr->points[final_pairs[i].first].z;
	    //temV2<<targetCloudPtr->points[final_pairs[i].second].x,targetCloudPtr->points[final_pairs[i].second].y,targetCloudPtr->points[final_pairs[i].second].z;
	    //temV1=R_update*temV1+t_update-temV2;
	    temV1=R_update*temV1+t_update;
	    //cout<<temV1;
	    Eigen::Matrix<float,3,6> Mk;
	    Mk.block(0,0,3,3).setIdentity();
	    cout<<Mk.block(0,0,3,3);
	    Mk.block(0,3,3,3)<<0, -temV1[1],-temV1[2],
					     temV1[2],temV1[0],0,
					     -temV1[1], 0,temV1[0];
	   //cout<<Mk;
	  */
	   temV1<<sourceCloudPtr->points[final_pairs[i].first].x,sourceCloudPtr->points[final_pairs[i].first].y,sourceCloudPtr->points[final_pairs[i].first].z;
	   temV1=R_update*temV1+t_update;
	    Eigen::Matrix<float,3,6> Mk;
	    Mk.block(0,0,3,3).setIdentity();
	    Mk.block(0,3,3,3)<<0, -temV1[2],temV1[1],
					     temV1[2],0,-temV1[0],
					     -temV1[1], temV1[0],0;
            InformationMatrix+=Mk.transpose()*Mk;    
	  }
	  InformationMatrix=(3*final_pairs.size()-3)*InformationMatrix/error_final;	  
	}
	else
	{
	  cout<<"error request for InformationMatrix, Because ICP has failed to register two point cloud"<<endl;
	}
	
      }
      
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr;
    /**
     * \memberof iter_maxnum protect algorithm from infinite loop once the iterative_counter exceed it.
     */
    int iter_maxnum;
    /**
     * \memberof convergence_counter_max protect algorithm from decetive convergence, only the times convergence criterion has been satified exceed it, the loop will end
     */
    int convergence_counter_max;
    /**
     * \memberof error_sigma is the average registration error that algorithm must reach at least.
     */
    float error_sigma;
    /**
     * \memberof error_belta is the error change threshold for each point. Only change small the average error of registration, the algorithm is treated as convergence.
     */
    float error_belta;

    double error_final;
    
    Eigen::Matrix3f R_update;
    Eigen::Vector3f t_update;
  };
  
  
}
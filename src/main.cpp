#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <openMVG/cameras/Camera_Common.hpp>
#include <math.h>
#include <time.h>
#include <omp.h> 

#include <unistd.h> 

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <kinect.hpp>
#include <create_directory.hpp>
#include <stdlib.h>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/property_map.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Timer.h>
#include <CGAL/Memory_sizer.h>
#include <CGAL/random_simplify_point_set.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>

#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/Shape_detection_3/Shape_detection_traits.h>
#include <tbb/tbb.h>

using namespace std;
using namespace libfreenect2;
using namespace Eigen;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::sfm;



typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Point> Point_set;


// Type declarations
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
// In Shape_detection_traits the basic types, i.e., Point and Vector types
// as well as iterator type and property maps, are defined.
typedef CGAL::Shape_detection_3::Shape_detection_traits
  <Kernel, Pwn_vector, Point_map, Normal_map>                Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection_3::Region_growing<Traits>      Region_growing;
typedef CGAL::Shape_detection_3::Plane<Traits>               Plane;
//using namespace openMVG::geodesy;

#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif
/*
int main()
{
	const char compute_matches[100] = "openMVG_main_ComputeMatches -i imgsave_out/matches/sfm_data.json -o imgsave_out/matches -g e";
	const char global_sfm[150] = "openMVG_main_GlobalSfM -i imgsave_out/matches/sfm_data.json -m imgsave_out/matches -o imgsave_out/reconstruction";
	const char structure_from_known_poses[200] = "openMVG_main_ComputeStructureFromKnownPoses -i imgsave_out/reconstruction/sfm_data.bin -m imgsave_out/matches -f imgsave_out/matches/matches.e.bin -o imgsave_out/reconstruction/robust.bin";
	const char sfm_to_pmvs[100] = "openMVG_main_openMVG2PMVS -i imgsave_out/reconstruction/robust.bin -o imgsave_out/reconstruction";
	const char pmvs2[100] = "pmvs2 imgsave_out/reconstruction/PMVS/ pmvs_options.txt PATCH PSET";
	if(create_directory())
		cout<<"Success Create!\n";
	else
		cout<<"Error Create!\n";
  	
	Kinect_camera cam; 	
	cam.image_describer_out();
	if(!cam.start())
		cout<<"Started already!\n";
	cam.frame_stream(1);
	//sleep(3);
	cam.close();
	
	cam.save_sfm_data();
	cout<<"11111111111111--------\n";
	std::system(compute_matches);
	cout<<"22222222222222--------\n";
	std::system(global_sfm);
	cout<<"33333333333333--------\n";
	std::system(structure_from_known_poses);
	cout<<"44444444444444--------\n";
	std::system(sfm_to_pmvs);
	cout<<"55555555555555--------\n";
	std::system(pmvs2);
	cout<<"66666666666666--------\n";
	cout<<"end!"<<endl;

	
	return 0;

}
*/



short cross(double X1,double Y1,double X2,double Y2,double X3,double Y3,double X4,double Y4,double &x,double &y)  
{  
     double k1,k2;  
     if((X1==X2)&&(Y1==Y2)&&(Y3==Y4)&&(X3==X4))  
     {  
         return 0;  
     }  
     if((X1==X2)&&(Y1==Y2))  
     {  
       if((X3==X4)&&(X1==X3))  
       {  
          x=X1;  
          y=Y1;  
          return 1;  
       }  
       else if(Y2==((X2-X3)*(Y4-Y3)/(X4-X3)+Y3))  
       {  
          x=X1;  
          y=Y1;  
          return 1;  
       }  
    }  
     if((Y3==Y4)&&(X3==X4))  
     {  
       if((X2==X1)&&(X1==X3))  
       {  
         x=X3;  
         y=Y3;  
         return 2;  
       }  
       else if(Y3==((X3-X1)*(Y2-Y1)/(X2-X1)+Y1))  
       {  
         x=X3;  
         y=Y3;  
         return 2;  
       }  
     }  
     if(X1!=X2)  
     {  
       k1=(Y2-Y1)/(X2-X1);  
       if(X3!=X4)  
       {  
         k2=(Y4-Y3)/(X4-X3);  
         if(k1==k2)  
         {   
             return 0;  
         }  
         x=(Y3-Y1-k2*X3+X1*k1)/(k1-k2);  
         y=k1*(x-X1)+Y1;  
         if(((X1-x)*(X2-x)<0||(X1-x)*(X2-x)==0)&&((Y1-y)*(Y2-y)<0||(Y1-y)*(Y2-y)==0))  
           return 1;  
         if(((X3-x)*(X4-x)<0||(X3-x)*(X4-x)==0)&&((Y3-y)*(Y4-y)<0||(Y3-y)*(Y4-y)==0))  
           return 2;  
         if((X3-x)*(X4-x)>0&&(Y3-y)*(Y4-y)>0&&(X1-x)*(X2-x)>0&&(Y1-y)*(Y2-y)>0)  
           return 3;  
      }  
      if(X3==X4)  
      {     
        x=X3;  
        y=k1*(X3-X1)+Y1;  
        return 1;  
      }  
     }  
     else  
     {  
       if(X3!=X4)  
       {  
         k2=(Y4-Y3)/(X4-X3);  
         x=double(X1);  
         y=k2*(X1-X3)+Y3;  
         return 2;  
       }  
       if(X3==X4)  
       {  
          return 0;  
       }  
     }  
     return 0;  
}  




void compute_transform(SfM_Data & sfm_data, int a, int b)
{
	Views::const_iterator iter = sfm_data.GetViews().begin();
	Pose3 pose_b, pose_a;
	if(a>b)
	{
		int i = 0;
		for(;i<b;i++)
			iter++;			
		pose_b = sfm_data.GetPoseOrDie(iter->second.get());	
		for(;i<a;i++)
			iter++; 	
		pose_a = sfm_data.GetPoseOrDie(iter->second.get());
	}
	else
	{
		int i = 0;
		for(;i<a;i++)
			iter++;			
		pose_a = sfm_data.GetPoseOrDie(iter->second.get());	
		for(;i<b;i++)
			iter++; 	
		pose_b = sfm_data.GetPoseOrDie(iter->second.get());
	}
	Mat3 rotation_a = pose_a.rotation().transpose();
  	Vec3 center_a = pose_a.center();
	Mat3 rotation_b = pose_b.rotation().transpose();
  	Vec3 center_b = pose_b.center();	
  	cout<<rotation_a<<endl<<rotation_b<<endl;
  	cv::Mat img_a = cv::imread(img_path_s+"/img"+to_string(a)+".bmp",-1);
	cv::Mat img_float_a(img_a.rows, img_a.cols, CV_32FC1, img_a.data);
	cv::Mat img_b = cv::imread(img_path_s+"/img"+to_string(b)+".bmp",-1);
	cv::Mat img_float_b(img_b.rows, img_b.cols, CV_32FC1, img_b.data);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
    int index_a1 = 0;
    int index_a2 = 512;
    int index_b1 = 0;
    int index_b2 = 512;
    Vec3 a1(0.051*rotation_a(0,0)+center_a[0],0,0.051*rotation_a(2,0)+center_a[2]*1.2);
    Vec3 b1(0.051*rotation_b(0,0)+center_b[0],0,0.051*rotation_b(2,0)+center_b[2]*1.2);
    float distance = sqrt((a1[0]-b1[0])*(a1[0]-b1[0])+(a1[2]-b1[2])*(a1[2]-b1[2]));
    float angle_a1 = acos(rotation_a(0,0));
    if((angle_a1>0 && rotation_a(2,0)<0) || (angle_a1<0 && rotation_a(2,0)<0))
    	angle_a1 = -angle_a1;
  	float angle_b1 = acos(rotation_b(0,0));
    if((angle_b1>0 && rotation_b(2,0)<0) || (angle_b1<0 && rotation_b(2,0)<0))
    	angle_b1 = -angle_b1; 
    float angle = abs(angle_b1-angle_a1)*180/M_PI;  
    cout<<"angle a1="<<angle_a1<<"\nangle b1="<<angle_b1<<endl;
    cout<<"distance: "<<distance<<"\nangle: "<<angle<<endl;		
    if(distance<0.05 && angle>1)
    {
    	float theta = angle/70*512;
    	if(angle_b1<angle_a1)
    	{
    		index_a1 = int(theta);
    		index_b2 = 512-index_a1;
    	}
    	else
    	{
    		index_b1 = int(theta);
    		index_a2 = 512-index_b1;
    	}
    	
    }	
    else if(angle<1)
    {
    	float theta = distance/5.8*512;
        if(angle_b1<angle_a1)
    	{
    		index_a1 = int(theta);
    		index_b2 = 512-index_a1;
    	}
    	else
    	{
			index_b1 = int(theta);
    		index_a2 = 512-index_b1;
    	}
    }
    cout<<index_a1<<" "<<index_a2<<" "<<index_b1<<" "<<index_b2<<endl;
    
    

    a1[0]=0.051*rotation_a(0,0);
    a1[2]=0.051*rotation_a(2,0);
    b1[0]=0.051*rotation_b(0,0);
    b1[2]=0.051*rotation_b(2,0);
    	float x,y,z;
    	const float cx=257.524, cy=208.875;
  		const float fx= 1/365.147, fy = 1/365.147;
       //赋值原始点云
        cloud_a->points.resize((index_a2-index_a1)*424);
    	int count_a = 0;
		for(int j = index_a1;j<index_a2; j++)
		{
			for(int k = 100;k<300; k++)
			{
  				float depth_val = img_float_a.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.2 || depth_val > 3.5)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else
  				{ 					
    				x = (j + 0.5 - cx) * fx * depth_val;
    				y = (k + 0.5 - cy) * fy * depth_val;
    				z = depth_val;  				
    				Vec3 vec1(y,x,z);
    				Vec3 vec2 = rotation_a*vec1+a1;  				
     				cloud_a->points[count_a].x =vec2[0];
    				cloud_a->points[count_a].y =vec2[1];
    				cloud_a->points[count_a].z =vec2[2];	
    				count_a++;			
  				}
			}
		}
	
		cloud_a->points.resize(count_a);
        cloud_b->points.resize((index_b2-index_b1)*424);
    	int count_b = 0;
		for(int j = index_b1;j<index_b2; j++)
		{
			for(int k = 100;k<300; k++)
			{
  				float depth_val = img_float_b.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.2 || depth_val > 3.5)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else
  				{ 					
    				x = (j + 0.5 - cx) * fx * depth_val;
    				y = (k + 0.5 - cy) * fy * depth_val;
    				z = depth_val;  				
    				Vec3 vec1(y,x,z);
    				Vec3 vec2 = rotation_b*vec1+b1;   				
     				cloud_b->points[count_b].x =vec2[0];
    				cloud_b->points[count_b].y =vec2[1];
    				cloud_b->points[count_b].z =vec2[2];	
    				count_b++;			
  				}
			}
		}
		cloud_b->points.resize(count_b);
		cout<<"count a : "<<count_a<<"\ncount b: "<<count_b<<endl;
		

	float center_dx = center_b[0]-center_a[0];
	float center_dz = center_b[2]-center_a[2];

	//int combine_count = 100000000;
	int combine_x = -1;
	int combine_z = -1;

	float step=0.04, start_x=0.8, start_z=1,squared_threshold;
	if(center_dx<center_dz)
		squared_threshold = center_dz*center_dz*0.16;
	else
		squared_threshold = center_dx*center_dx*0.16;
	if(distance<0.05)
	{
		start_x = -0.05;
		start_z = -0.05;
		step = 0.01;
		center_dx = 1;
		center_dz = 1;
		squared_threshold = 0.008;
	}

	float cost_value = 10000000;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
 	kdtree.setInputCloud (cloud_b);


    time_t start, finish;  
    time(&start); 
 	int i=0, j=0;
 	float cost_array[11][11];

 	//#pragma omp parallel for
	for(i=0;i<11;i++)
	{
		//#pragma omp parallel for 
		for(j=0;j<11;j++)
		{
			float cost = 0;
			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			transform(0,3)=center_dx*(start_x+i*step);
			transform(2,3)=center_dz*(start_z+j*step);
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::transformPointCloud (*cloud_a, *transformed_cloud, transform);
  			#pragma omp parallel for reduction(+:cost)
			for(int k = 0; k<count_a;k+=10)
			{
				pcl::PointXYZ searchPoint;
 				std::vector<int> pointIdxNKNSearch;
  				std::vector<float> pointNKNSquaredDistance;
 				searchPoint= transformed_cloud->points[k];
  				if ( kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) != 0 )
  				{
  					if(pointNKNSquaredDistance[0]<=squared_threshold)
						cost = cost + pointNKNSquaredDistance[0];
				}
			}
	
			if(cost<cost_value)
			{
				cost_value = cost;
				combine_x = i;
				combine_z = j;
			}
			

			//cout<<"i="<<i<<"  j="<<j<<"  "<<cost<<endl;
		}
	}
	time(&finish);
	double duration = difftime(finish, start);  
	cout << "--> time: " << duration << " s" << endl;  

	cout<<"---\ni="<<combine_x<<" j="<<combine_z<<" "<<cost_value<<endl;
}



int main()
{		
		  
	const float cx=257.524, cy=208.875;
  	const float fx= 1/365.147, fy = 1/365.147;
	const Vec3 T(0.0542155,0,0);
/* 
	const float cx=259.9229, cy=209.0684;
  	const float fx= 1/361.8275, fy = 1/362.7217;
  	const Vec3 T(0.051,0,0);
*/
	int count=0;
	float x,y,z;
	
	SfM_Data sfm_data;
	Load(sfm_data, "imgsave_out/reconstruction/sfm_data.bin", ESfM_Data(ALL));
	Views::const_iterator iter = sfm_data.GetViews().begin();
	vector<Vec3> coordinate;
	int image_count = sfm_data.views.size();
	cout<<"image count: "<<image_count<<endl;
	int pose_count = sfm_data.poses.size();
	cout<<"pose count"<<pose_count<<endl;

	std::vector<Point> points;
	const int nb_neighbors = 10; 
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(512*424*pose_count);
	int dp= 0;

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudadd(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::io::loadPLYFile("out1.ply", *cloud1);  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);  
    pcl::io::loadPLYFile("out2.ply", *cloud2); 
    ifstream fin("imgsave_out/reconstruction/PMVS/vis.dat");
    string s;
    fin>>s;
    cout<<s<<endl;
    int pmvs_count;
    fin>>pmvs_count;
    cout<<pmvs_count<<endl;
    Eigen::MatrixXd pmvs_mat=Eigen::MatrixXd::Identity(pmvs_count, pmvs_count);  //邻接矩阵
    for(int i = 0; i<pmvs_count;i++)
    {
    	int index, num;
    	fin>>index>>num;
    	int content;
    	for(int j = 0; j<num; j++)
    	{
    		fin>>content;
    		pmvs_mat(index,content)=1;
    		pmvs_mat(content,index)=1;
    	}
    }
    cout<<pmvs_mat<<endl;

/*

 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
 	icp.setMaxCorrespondenceDistance (0.05);
// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (100);
// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-10);
// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (0.001);
    icp.setInputCloud(cloud1);
    icp.setInputTarget(cloud2);

    //Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm
    pcl::PointCloud<pcl::PointXYZ> Final;

    //Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
    icp.align(Final);

    //Return the state of convergence after the last align run. 
    //If the two PointClouds align correctly then icp.hasConverged() = 1 (true). 
    std::cout << "has converged: " << icp.hasConverged() <<std::endl;

    //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target) 
    std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
    std::cout << "----------------------------------------------------------"<< std::endl;

    //Get the final transformation matrix estimated by the registration method. 
    std::cout << icp.getFinalTransformation() << std::endl;

*/

	
	compute_transform(sfm_data,0,1);
/*	
	iter++;
	for(int i=1;i<3;i++,iter++)
	{
		cout<<"---------------\n";
		cv::Mat img = cv::imread(img_path_s+"/img"+to_string(i)+".bmp",-1);
		cv::Mat img_float(img.rows, img.cols, CV_32FC1, img.data);
  		View * view = iter->second.get();
  		if(!sfm_data.IsPoseAndIntrinsicDefined(view))
  			continue;
  		Pose3 pose = sfm_data.GetPoseOrDie(view);		
  		Mat3 rotation = pose.rotation().transpose();
  		Vec3 center = pose.center();
  		cout<<i<<endl;
  		cout<<"rotation: \n"<<rotation<<endl;
  		cout<<"center: "<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
  		center[0] = 0.051*rotation(0,0)+center[0]*0.8;
  		center[1] = 0;
  		center[2] = 0.051*rotation(2,0)+center[2]*1.15;
  		std::vector<Point> temp;
		for(int j = 0;j<img_float.cols; j++)
		{
			for(int k = 0;k<img_float.rows; k++)
			{
  				float depth_val = img_float.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.5 || depth_val > 3.5)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else
  				{

  					
    				x = (j + 0.5 - cx) * fx * depth_val;
    				y = (k + 0.5 - cy) * fy * depth_val;
    				z = depth_val;  				
    				Vec3 vec1(y,x,z);
    				Vec3 vec2 = rotation*(vec1)+center;   				
     				cloud->points[count].x =vec2[0];
    				cloud->points[count].y =vec2[1];
    				cloud->points[count].z =vec2[2];
    				   				
    				//Vec3 vec1(x,y,z);
    				//Vec3 vec2 = vec1;
    				coordinate.push_back(vec2);
    				count++;   				
    				//Point_with_normal tp(Point(vec2[0],vec2[1],vec2[2]), Vector(1,0,0));
    				//points.push_back(tp);
    				points.push_back (Point (vec2[0], vec2[1], vec2[2]));					
  				}
			}
		}
		if(i==1)
			dp=count;

	}


	
	

	
	ofstream out_origin1("out1.ply");

	out_origin1<<"ply\nformat ascii 1.0\nelement vertex "<<dp<<endl;
	out_origin1<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	for(int i= 0 ;i<dp;i++)
	{
		out_origin1<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<endl;
	}
	out_origin1.close();
	
	ofstream out_origin2("out2.ply");
	out_origin2<<"ply\nformat ascii 1.0\nelement vertex "<<count-dp<<endl;
	out_origin2<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	for(int i= dp ;i<count;i++)
	{
		out_origin2<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<endl;
	}
	out_origin2.close();
*/
/*
  std::vector<Point> output;
  //parameters
  const double retain_percentage = 3;   // percentage of points to retain.
  const double neighbor_radius = 0.1;   // neighbors size.
  CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>
    (points, std::back_inserter(output),
     CGAL::parameters::select_percentage(retain_percentage).
     neighbor_radius (neighbor_radius));
  
  std::ofstream out("wlop.ply");
  CGAL::write_ply_points(out, output);
*/

/*
	
	cout<<"preliminary: "<<count<<endl;
	pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // 设置体素栅格的大小为 1x1x1cm
    filter.setLeafSize(0.01f, 0.01f, 0.01f);
    filter.filter(*filteredCloud);
	cout<<"after simplify: "<<filteredCloud->width<<endl;
	
	ofstream out_cloud("outcloud.ply");
	int all_cloud = filteredCloud->width;
	out_cloud<<"ply\nformat ascii 1.0\nelement vertex "<<filteredCloud->width<<endl;
	out_cloud<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	for(int i= 0 ;i<all_cloud;i++)
	{
		out_cloud<<filteredCloud->points[i].x<<" "<<filteredCloud->points[i].y<<" "<<filteredCloud->points[i].z<<endl;
	}
	out_cloud.close();
*/


	/*
	cout<<"preliminary: "<<points.size()<<endl;
	std::vector<Point>::iterator remove = CGAL::random_simplify_point_set(points.begin(), points.end(), CGAL::Identity_property_map<Point>(), 50);
	points.erase(remove, points.end());
	cout<<"after random simplify: "<<points.size()<<endl;

	ofstream out("simple.ply");
	CGAL::write_ply_points(out, points.begin(), points.end());
	out.close();	
	
	// Construct the mesh in a scale space.  

	CGAL::Timer t;
  	t.start();
  	Reconstruction reconstruct (points.begin(), points.end());
  	reconstruct.increase_scale(1);
  	reconstruct.reconstruct_surface();
  	std::cerr << "done in " << t.time() << " sec." << std::endl;
  	std::ofstream out1 ("out.off");
  	out1 << reconstruct;
  	std::cerr << "Done." << std::endl;
	*/


	return 0;
}


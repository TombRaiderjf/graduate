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


#include <openMVG/features/feature.hpp>
#include <openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp>
#include <openMVG/features/svg_features.hpp>
#include <openMVG/image/image_io.hpp>
#include <openMVG/image/image_concat.hpp>
#include <openMVG/matching/regions_matcher.hpp>
#include <openMVG/matching/svg_matches.hpp>
#include <openMVG/matching/kvld/kvld.h>
#include <openMVG/matching/kvld/kvld_draw.h>
#include <openMVG/matching/regions_matcher.hpp>
#include <openMVG/matching/svg_matches.hpp>

#include <openMVG/third_party/stlplus3/filesystemSimplified/file_system.hpp>
#include <openMVG/third_party/vectorGraphics/svgDrawer.hpp>

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

	/*	
	ofstream out_a("out_a.ply");
	out_a<<"ply\nformat ascii 1.0\nelement vertex "<<count_a<<endl;
	out_a<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	for(int i= 0 ;i<count_a;i++)
	{
		out_a<<cloud_a->points[i].x<<" "<<cloud_a->points[i].y<<" "<<cloud_a->points[i].z<<endl;
	}
	out_a.close();
	
	ofstream out_b("out_b.ply");
	out_b<<"ply\nformat ascii 1.0\nelement vertex "<<count_b<<endl;
	out_b<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	for(int i= 0 ;i<count_b;i++)
	{
		out_b<<cloud_b->points[i].x<<" "<<cloud_b->points[i].y<<" "<<cloud_b->points[i].z<<endl;
	}
	out_b.close();
	*/


void compute_border(int a, int b, cv::Mat& img_a, cv::Mat& img_b, Kinect_camera& cam, cv::Mat& gray_a, cv::Mat& gray_b)
{

	cv::Mat img_float_a(img_a.rows, img_a.cols, CV_32FC1, img_a.data);
	cv::Mat img_float_b(img_b.rows, img_b.cols, CV_32FC1, img_b.data);

 
string path_a = "kvldOut"+to_string(a)+to_string(b)+"/05_Left-K-VLD-MASK.jpg";
string path_b = "kvldOut"+to_string(a)+to_string(b)+"/06_Right-K-VLD-MASK.jpg";
cv::Mat Mat_a = cv::imread(path_a,cv::IMREAD_GRAYSCALE);
cv::Mat Mat_b = cv::imread(path_b,cv::IMREAD_GRAYSCALE);

cout<<a<<" "<<b<<"------------------\n";

cv::Mat Mat_a1;
cvtColor(Mat_a, Mat_a1, CV_GRAY2RGBA);

Frame undistorted_a(512, 424, 4), registered_a(512, 424, 4);
Frame rgb_a(1920, 1080, 4, Mat_a1.data), depth_a(512, 424, 4, img_a.data);
cam.registration->apply(&rgb_a, &depth_a, &undistorted_a, &registered_a);
cv::Mat regMat_a((int)registered_a.height, (int)registered_a.width, CV_8UC4, registered_a.data);
//cv::Mat gray_a;

cvtColor(regMat_a, gray_a, CV_RGBA2GRAY);
//cv::imshow("a", gray_a);
//cv::waitKey();


cv::Mat Mat_b1;
cvtColor(Mat_b, Mat_b1, CV_GRAY2RGBA);

Frame undistorted_b(512, 424, 4), registered_b(512, 424, 4);
Frame rgb_b(1920, 1080, 4, Mat_b1.data), depth_b(512, 424, 4, img_b.data);
cam.registration->apply(&rgb_b, &depth_b, &undistorted_b, &registered_b);
cv::Mat regMat_b((int)registered_b.height, (int)registered_b.width, CV_8UC4, registered_b.data);
//cv::Mat gray_b;

cvtColor(regMat_b, gray_b, CV_RGBA2GRAY);
//cv::imshow("b", gray_b);
//cv::waitKey();

}


Vec3 compute_transform(SfM_Data & sfm_data, int a, int b, Kinect_camera& cam, float& weight)
{
	weight = 0;
	time_t start, finish;  
    time(&start); 
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
  	//cout<<rotation_a<<endl<<rotation_b<<endl;
  	cv::Mat img_a = cv::imread(img_path_s+"/img"+to_string(a)+".bmp",-1);
	cv::Mat img_float_a(img_a.rows, img_a.cols, CV_32FC1, img_a.data);
	cv::Mat img_b = cv::imread(img_path_s+"/img"+to_string(b)+".bmp",-1);
	cv::Mat img_float_b(img_b.rows, img_b.cols, CV_32FC1, img_b.data);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
    Vec3 a1(0.051*rotation_a(0,0)+center_a[0],0,0.051*rotation_a(2,0)+center_a[2]);
    Vec3 b1(0.051*rotation_b(0,0)+center_b[0],0,0.051*rotation_b(2,0)+center_b[2]);
    float distance = sqrt((a1[0]-b1[0])*(a1[0]-b1[0])+(a1[2]-b1[2])*(a1[2]-b1[2]));
	cout<<"distance = "<<distance<<endl;
 
 
 	cv::Mat gray_a(424,512,CV_8U), gray_b(424,512,CV_8U);
    compute_border(a,b,img_a,img_b,cam,gray_a,gray_b);
    cout<<"aaaaaaaaaaaa\n";
    /*
    cv::imshow("a",gray_a);
    cv::waitKey();
    cv::imshow("b",gray_b);
    cv::waitKey();
   	*/ 
   	
    a1[0]=0.051*rotation_a(0,0);
    a1[2]=0.051*rotation_a(2,0);
    b1[0]=0.051*rotation_b(0,0);
    b1[2]=0.051*rotation_b(2,0);
    	
    float x,y,z;
    const float cx=257.524, cy=208.875;
  	const float fx= 1/365.147, fy = 1/365.147;
  		
       //赋值原始点云
        cloud_a->points.resize(512*424);
    	int count_a = 0;
		for(int j = 0;j<424; j++)
		{
			for(int k = 0;k<512; k++)
			{
  				float depth_val = img_float_a.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.5 || depth_val > 3.5)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else if(gray_a.at<uchar>(j,k) == 255)
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
		if(count_a<5000)
			return Vec3(0,0,0);
	
		cloud_a->points.resize(count_a);
		//cout<<"sum of a: "<<count_a<<endl;
        cloud_b->points.resize(512*424);
    	int count_b = 0;
		for(int j = 0;j<424; j++)
		{
			for(int k = 0;k<512; k++)
			{
  				float depth_val = img_float_b.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.5 || depth_val > 3.5)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else if(gray_b.at<uchar>(j,k)==255)
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
		if(count_b<5000)
			return Vec3(0,0,0);
		cloud_b->points.resize(count_b);
		cout<<"count a : "<<count_a<<"\ncount b: "<<count_b<<endl;
		//赋值原始点云
	if((count_a*1.3< count_b) || (count_b*1.3<count_a))
		return Vec3(0,0,0);
	float center_dx = center_b[0]-center_a[0];
	float center_dz = center_b[2]-center_a[2];
	cout<<"x distance = "<<center_dx<<" z distance = "<<center_dz<<endl;
//做ICP配准，得到准确的平移向量		
			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			transform(0,3)= -center_dx;
			transform(2,3)= -center_dz;
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::transformPointCloud (*cloud_a, *transformed_cloud, transform);
	pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //创建ICP的实例类
 	icp.setInputSource(transformed_cloud);
	icp.setInputTarget(cloud_b);
	if(distance<0.3)
		icp.setMaxCorrespondenceDistance(0.1);  
	else
 		icp.setMaxCorrespondenceDistance(distance*0.33);  
	icp.setTransformationEpsilon(1e-10); 
	icp.setEuclideanFitnessEpsilon(0.001); 
	icp.setMaximumIterations(100);   
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl; //输出最终的变换矩阵（4x4）
	if(icp.getFitnessScore()>0.002)
		return Vec3(0,0,0);
	weight =icp.getFitnessScore(); 
	Eigen::Matrix4f matrix = icp.getFinalTransformation();
 	std::cout << matrix << std::endl;
//做ICP配准，得到准确的平移向量


	time(&finish);
	double duration = difftime(finish, start);  
	cout << "--> time: " << duration << " s" << endl;  

	//Vec3 res(center_dx*(start_x+combine_x*step_x),0,center_dz*(start_z+combine_z*step_z));
	Vec3 res(center_dx-matrix(0,3),-matrix(1,3),center_dz-matrix(2,3));
	return res;
	
	//return Vec3(0,0,0);
}



int main()
{		

Kinect_camera cam; 	
/*
	const char compute_matches[100] = "openMVG_main_ComputeMatches -i imgsave_out/matches/sfm_data.json -o imgsave_out/matches -g e";
	const char global_sfm[150] = "openMVG_main_GlobalSfM -i imgsave_out/matches/sfm_data.json -m imgsave_out/matches -o imgsave_out/reconstruction";
	const char structure_from_known_poses[200] = "openMVG_main_ComputeStructureFromKnownPoses -i imgsave_out/reconstruction/sfm_data.bin -m imgsave_out/matches -f imgsave_out/matches/matches.e.bin -o imgsave_out/reconstruction/robust.bin";
	if(create_directory())
		cout<<"Success Create!\n";
	else
		cout<<"Error Create!\n";
  	
	
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

*/
	if(!cam.start())
		cout<<"Started already!\n";
cam.close();


		  
	const float cx=257.524, cy=208.875;
  	const float fx= 1/365.147, fy = 1/365.147;
	const Vec3 T(0.0542155,0,0);

	int count=0;
	float x,y,z;
	
	SfM_Data sfm_data;
	Load(sfm_data, "imgsave_out/reconstruction/robust.bin", ESfM_Data(ALL));
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

	Vec3** trans = new Vec3*[image_count];
	for(int i = 0; i<image_count;i++)
		trans[i] = new Vec3[image_count];


	trans[0][0]=Vec3(0,0,0);
	/*
	cv::Mat image = cv::Mat::zeros(1080, 400, CV_8UC3);  
	image.setTo(cv::Scalar(0, 0, 0));  
	for(int i = 0;i<image_count;i++)
	{	
		string name = img_path_s+"/img"+to_string(i)+".jpg";
		cv::Mat small = cv::imread(name);
		cv::Rect roi_rect1 = cv::Rect(0, 0, 400, 1080);  
		image.copyTo(small(roi_rect1)); 
		cv::Rect roi_rect2 = cv::Rect(1520, 0, 400, 1080);  
		image.copyTo(small(roi_rect2));  
		string name1 =  img_path_s+"/img0"+to_string(i)+".jpg";
		cv::imwrite(name1,small);
	}
	
	
	for(int i = 0; i<image_count;i++)
		for(int j = (i+1);j<image_count;j++)
		{
			if(pmvs_mat(i,j))
			{
				string jpg_filenameL = img_path_s+"/img0"+to_string(i)+".jpg";
  				string jpg_filenameR = img_path_s+"/img0"+to_string(j)+".jpg";
				string str = "./openMVG_sample_features_kvld -i "+jpg_filenameL+" -j "+jpg_filenameR+" -o ./kvldOut"+to_string(i)+to_string(j);
  				const char* compute_matches = str.c_str();
  				int status;  

    			status = std::system(compute_matches);
    			if(status < 0)  
    			{  
        			printf("cmd: \t error: %s",strerror(errno)); // 这里务必要把errno信息输出或记入Log  
        			continue;  
    			}  
      
    			if(WIFEXITED(status))  
    			{  
        			printf("normal termination, exit status = %d\n", WEXITSTATUS(status)); //取得cmdstring执行结果  
    			}  
    			else if(WIFSIGNALED(status))  
    			{  
        			printf("abnormal termination,signal number =%d\n", WTERMSIG(status)); //如果cmdstring被信号中断，取得信号值  
    			}  
    			else if(WIFSTOPPED(status))  
    			{  
        			printf("process stopped, signal number =%d\n", WSTOPSIG(status)); //如果cmdstring被信号暂停执行，取得信号值  
    			}  
			}
		}
	
	
	cout<<"End computing--------------\n";
	*/
	
	
	for(int i = 0; i<image_count;i++)
	{
		for(int j = (i+1);j<image_count;j++)
		{
			float weight = -1;
			trans[i][j] = compute_transform(sfm_data,i,j,cam, weight);
			pmvs_mat(i,j) = weight;
		}
	}
	
	cout<<"Done! weight computed!\n"<<pmvs_mat<<endl;
	
	
/*	
	
{	
		cv::Mat img = cv::imread(img_path_s+"/img0.bmp",-1);
		cv::Mat img_float(img.rows, img.cols, CV_32FC1, img.data);
  		View * view = iter->second.get();
  		Pose3 pose = sfm_data.GetPoseOrDie(view);		
  		Mat3 rotation = pose.rotation().transpose();
  		Vec3 center = pose.center();
  		cout<<"rotation: \n"<<rotation<<endl;
  		cout<<"center: "<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
  		center[0] = 0.051*rotation(0,0);
  		center[1] = 0;
  		center[2] = 0.051*rotation(2,0);
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
    				coordinate.push_back(vec2);
    				count++;   	
    				//points.push_back(Point(vec2[0],vec2[1],vec2[2]));								
  				}
			}
		}	
}

	iter++;
	for(int i = 1; i<image_count ;i++, iter++)
	{
		cout<<i<<endl;
		float weight=-1;
		Vec3 tran=compute_transform(sfm_data,0,i,cam, weight);
		if(!weight)
			continue;
		cv::Mat img = cv::imread(img_path_s+"/img"+to_string(i)+".bmp",-1);
		cv::Mat img_float(img.rows, img.cols, CV_32FC1, img.data);
  		View * view = iter->second.get();
  		if(!sfm_data.IsPoseAndIntrinsicDefined(view))
  			continue;
  		Pose3 pose = sfm_data.GetPoseOrDie(view);		
  		Mat3 rotation = pose.rotation().transpose();
  		Vec3 center = pose.center();
  		
  		//cout<<"rotation: \n"<<rotation<<endl;
  		//cout<<"center: "<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
  		center[0] = 0.051*rotation(0,0);
  		center[1] = 0;
  		center[2] = 0.051*rotation(2,0);
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
    				Vec3 vec2 = rotation*(vec1)+center+tran;   				
     				cloud->points[count].x =vec2[0];
    				cloud->points[count].y =vec2[1];
    				cloud->points[count].z =vec2[2];
    				coordinate.push_back(vec2);
    				//points.push_back(Point(vec2[0],vec2[1],vec2[2]));
    				count++;   				
				
  				}
			}
		}
		
	}
*/


/*

	for(int i=0;i<2;i++,iter++)
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
  		center[0] = 0.051*rotation(0,0);
  		center[1] = 0;
  		center[2] = 0.051*rotation(2,0);
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
		if(i==0)
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



	cloud->points.resize(count);
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
		points.push_back(Point(filteredCloud->points[i].x,filteredCloud->points[i].y,filteredCloud->points[i].z));
	}
	out_cloud.close();
/*
  std::vector<Point> output;
  //parameters
  const double retain_percentage = 20;   // percentage of points to retain.
  const double neighbor_radius = 0.1;   // neighbors size.
  CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>
    (points, std::back_inserter(output),
     CGAL::parameters::select_percentage(retain_percentage).
     neighbor_radius (neighbor_radius));
  
  std::ofstream out("wlop.ply");
  CGAL::write_ply_points(out, output);
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


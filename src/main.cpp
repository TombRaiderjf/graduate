#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <openMVG/cameras/Camera_Common.hpp>


#include <unistd.h> 


#include <kinect.hpp>
#include <create_directory.hpp>
#include <stdlib.h>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>


#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/point_types.h> //各种点类型
#include <pcl/registration/icp.h> //ICP(iterative closest point)配准
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>



#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/property_map.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/Point_set_3/IO.h>


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


#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


//using namespace openMVG::geodesy;

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
int main()
{				  
	const float cx=257.524, cy=208.875;
  	const float fx= 1/365.147, fy = 1/365.147;
	ofstream fout("model.ply");
	fout<<"ply\nformat ascii 1.0\n";
	int count=0;
	float x,y,z;
	
	SfM_Data sfm_data;
	Load(sfm_data, "imgsave_out/reconstruction/robust.bin", ESfM_Data(ALL));
	Views::const_iterator iter = sfm_data.GetViews().begin();
	vector<Vec3> coordinate;



	//Mat3 rot;
	//rot<<0.997248,-0.0638613,-0.037456,0.0618396,0.996699,-0.0526039,0.0408809,0.0501313,0.997905;
	//Vec3 trans(-0.0591987,-0.043131,0.0136954);

	int dp=0;
	
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>); //创建输入点云（指针）
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>); //创建输出/目标点云（指针）
  	
  	cloud_in->width = 512;
  	cloud_in->height = 424;
  	cloud_in->is_dense = false;
  	cloud_in->points.resize (cloud_in->width * cloud_in->height); //变形，无序
  	
  	cloud_out->width = 512;
  	cloud_out->height = 424;
  	cloud_out->is_dense = false;
  	cloud_out->points.resize (cloud_in->width * cloud_in->height); //变形，无序
	
	
	std::vector<Point> points;
	
	for(int i=0;i<2;i++,iter++)
	{
		cout<<"---------------\n";
		cv::Mat img = cv::imread(img_path_s+"/img"+to_string(i)+".bmp",-1);
		cv::Mat img_float(img.rows, img.cols, CV_32FC1, img.data);
  		View * view = iter->second.get();
  		Pose3 pose = sfm_data.GetPoseOrDie(view);		
  		Mat3 rotation = pose.rotation().transpose();
  		Vec3 center = pose.center();
  		cout<<"rotation: \n"<<rotation<<endl;
 		//Vec3 center1(center[1],center[0],0);

  		cout<<"center: "<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
  		if(i==0)
		for(int j = 0;j<img_float.cols; j++)
		{
			for(int k = 0;k<img_float.rows; k++)
			{
  				float depth_val = img_float.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.001)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else
  				{

  					
    				x = (j + 0.5 - cx) * fx * depth_val;
    				y = (k + 0.5 - cy) * fy * depth_val;
    				z = depth_val;
    				
    				
    				//cloud_in->points[count].x =x;
    				//cloud_in->points[count].y =y;
    				//cloud_in->points[count].z =z;
    				
    				int index = j*512+k;
    				cloud_in->points[index].x =x;
    				cloud_in->points[index].y =y;
    				cloud_in->points[index].z =z;
    				
    				
    				Vec3 vec1(y,x,z);
    				Vec3 vec2 = rotation*vec1+center;
    				
    				
    				//Vec3 vec1(x,y,z);
    				//Vec3 vec2 = vec1;
    				coordinate.push_back(vec2);
    				count++;
    				
    				points.push_back (Point (x, y, z));

  				}
			}
		}
		else
		for(int j = 0;j<img_float.cols; j++)
		{
			for(int k = 0;k<img_float.rows; k++)
			{
  				float depth_val = img_float.at<float>(j,k)/1000.0f; //scaling factor, so that value of 1 is one meter.
  				if (isnan(depth_val) || depth_val <= 0.001)
  				{
    			//depth value is not valid
    				continue;
  				}
 				else
  				{

  					
    				x = (j + 0.5 - cx) * fx * depth_val;
    				y = (k + 0.5 - cy) * fy * depth_val;
    				z = depth_val;
    				
    				/*
    				cloud_out->points[count-dp].x =x;
    				cloud_out->points[count-dp].y =y;
    				cloud_out->points[count-dp].z =z;
    				
    				
    				Vec3 vec1(y,x,z);
    				Vec3 vec2 = rotation*vec1+center;
    				//Vec3 vec1(x,y,z);
    				//Vec3 vec2 = vec1;
    				coordinate.push_back(vec2);
    				count++;
    				*/

  				}
			}
		}

		
		if(dp==0)
			dp=count;
		
	}
	
	/*
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
  	icp.setInputCloud(cloud_in); //设置输入点云
  	icp.setInputTarget(cloud_out); //设置目标点云（输入点云进行仿射变换，得到目标点云）
  	pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
  //进行配准，结果存储在Final中
  	icp.align(Final); 
  //输出ICP配准的信息（是否收敛，拟合度）
  	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  	icp.getFitnessScore() << std::endl;
  //输出最终的变换矩阵（4x4）
  	std::cout << icp.getFinalTransformation() << std::endl;
  	*/
  	
  	
  	
  // Removes outliers using erase-remove idiom.
  // The Identity_property_map property map can be omitted here as it is the default value.
  	const int nb_neighbors = 24; // considers 24 nearest neighbor points
  // Estimate scale of the point set with average spacing
 	const double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>
    (points.begin(), points.end(), nb_neighbors);
    cout<<"average spacing:"<<average_spacing<<endl;
  // FIRST OPTION //
  // I don't know the ratio of outliers present in the point set
  	std::vector<Point>::iterator first_to_remove
    = CGAL::remove_outliers(points.begin(), points.end(),
                            CGAL::Identity_property_map<Point>(),
                            nb_neighbors,
                            100.,                  // No limit on the number of outliers to remove
                            2. * average_spacing); // Point with distance above 2*average_spacing are considered outliers
  	std::cerr << (100. * std::distance(first_to_remove, points.end()) / (double)(points.size()))
            << "% of the points are considered outliers when using a distance threshold of "
            << 2. * average_spacing << std::endl;
  	points.erase(first_to_remove, points.end());
  	std::vector<Point>(points).swap(points);
  	cout<<points.size()<<endl;
  	
  	//WLOP
  	std::vector<Point> output;
  	const double retain_percentage = 20;   // percentage of points to retain.
  	const double neighbor_radius = 0.05;   // neighbors size.
  	CGAL::wlop_simplify_and_regularize_point_set
                          <Concurrency_tag>
                          (points.begin(), 
                           points.end(),
                           std::back_inserter(output),
                           retain_percentage,
                           average_spacing*5
                           );
  	cout<<output.size()<<endl;
	std::ofstream out ("out.ply");
	/*
  	out<<"ply\nformat ascii 1.0\n";
  	out<<"element vertex "<<output.size()<<std::endl;
	out<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	int output_size=output.size();
	for(int i=0;i<output_size;i++)
		out<<output[i].x<<" "<<output[i].y<<" "<<output[i].z<<endl;
	out.close();
	*/
	
	CGAL::write_ply_points(out, output.begin(), output.end());
  	
  	/*
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_re (new pcl::PointCloud<pcl::PointXYZ>); 		  		
  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud_in);
  	sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	sor.filter (*cloud_re);
  		
  	ofstream fout2("model2.ply");
  	fout2<<"ply\nformat ascii 1.0\n";
  	fout2<<"element vertex "<<cloud_re->size()<<std::endl;
	fout2<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	int after = cloud_re->size();
	for(int i = 0;i<after;i++)
		fout2<<cloud_re->points[i].x<<" "<<cloud_re->points[i].y<<" "<<cloud_re->points[i].z<<endl;
		
	fout2.close();
  	*/
  	
  	
  	ofstream fout1("model1.ply");
  	fout1<<"ply\nformat ascii 1.0\n";
  	fout1<<"element vertex "<<count-dp<<std::endl;
	fout1<<"property float x\nproperty float y\nproperty float z\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nend_header\n";
  	
	
	fout<<"element vertex "<<dp<<std::endl;
	fout<<"property float x\nproperty float y\nproperty float z\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nend_header\n";
	//fout<<"property float x\nproperty float y\nproperty float z\nend_header\n";
	
	int rgb[3]={0,255,0};
	for(int i=0;i<dp;i++)
	{
		fout<<coordinate[i][0]<<" "<<coordinate[i][1]<<" "<<coordinate[i][2]<<" "<<rgb[0]<<" "<<rgb[1]<<" "<<rgb[2]<<endl;
	}
	rgb[1]=0;
	rgb[2]=255;
	
	for(int i=dp;i<count;i++)
	{
		fout1<<coordinate[i][0]<<" "<<coordinate[i][1]<<" "<<coordinate[i][2]<<" "<<rgb[0]<<" "<<rgb[1]<<" "<<rgb[2]<<endl;
	}
	
	
	/*
	for(int i = 0;i<count;i++)
	{
		fout<<coordinate[i][0]<<" "<<coordinate[i][1]<<" "<<coordinate[i][2]<<endl;
	}
	*/
	
	fout1.close();
	fout.close();
	/*
	for(int i = 0; i<10;i++)
		for(int j = 0;j<10; j++)
			std::cout<<img1.at<float>(i,j)<<" ";
	cv::Mat img2;
	normalize(img1,img2,1, 0,  cv::NORM_MINMAX );
	cv::imshow("a",img2);
	cv::waitKey();
	*/
	
	return 0;
}



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
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel>    Reconstruction;
typedef Reconstruction::Facet_const_iterator                   Facet_iterator;


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
	int dp=0;
	
	std::vector<Point> points;
	const int nb_neighbors = 10; 

	for(int i=0;i<image_count;i++,iter++)
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
  		//cout<<"rotation: \n"<<rotation<<endl;
 		//Vec3 center1(center[1],center[0],0);
  		//cout<<"center: "<<center[0]<<" "<<center[1]<<" "<<center[2]<<endl;
  		//if(i==0)
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

  					
    				x = (j + 0.5 - cx) * fx * depth_val * 1.19;
    				y = (k + 0.5 - cy) * fy * depth_val *1.19;
    				z = depth_val*1.19;

    				/*
    				int index = j*512+k;
    				cloud_in->points[index].x =x;
    				cloud_in->points[index].y =y;
    				cloud_in->points[index].z =z;
    				*/
    				
    				Vec3 vec1(y,x,z);
    				Vec3 vec2 = rotation*vec1+center;
    				
    				
    				//Vec3 vec1(x,y,z);
    				//Vec3 vec2 = vec1;
    				coordinate.push_back(vec2);
    				count++;
    				
    				points.push_back (Point (vec2[0], vec2[1], vec2[2]));

  				}
			}
		}

		/*
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
    				

  				}
			}
		}

		
		if(dp==0)
			dp=count;
		*/
	}
	 	
	cout<<"preliminary: "<<points.size()<<endl;
	std::vector<Point>::iterator remove = CGAL::random_simplify_point_set(points.begin(), points.end(), CGAL::Identity_property_map<Point>(), 90);
	points.erase(remove, points.end());
	cout<<"after random simplify: "<<points.size()<<endl;
	/*
	std::vector<Point>::iterator first_to_remove
    = CGAL::remove_outliers(points.begin(), points.end(),
                            CGAL::Identity_property_map<Point>(),
                            nb_neighbors,
                            100.,      
                            0.05);
	points.erase(first_to_remove, points.end());
	
	int outline_count = points.size();
	cout<<"after outliner removal: "<<outline_count<<endl;	
	*/
	
	
	// Construct the mesh in a scale space.  
	CGAL::Timer t;
  	t.start();
  	Reconstruction reconstruct (points.begin(), points.end());
  	reconstruct.increase_scale(4);
  	reconstruct.reconstruct_surface();
  	std::cerr << "done in " << t.time() << " sec." << std::endl;
  	std::ofstream out1 ("out.off");
  	out1 << reconstruct;
  	std::cerr << "Done." << std::endl;
	
	ofstream out("simple.ply");
	CGAL::write_ply_points(out, points.begin(), points.end());
	out.close();

/*	
	cout<<"average spacing: "<<average_spacing<<endl;
	double cell_size = 0.01;
  	points.erase(CGAL::grid_simplify_point_set(points.begin(), points.end(), cell_size),
               points.end());
 
  	std::vector<Point>(points).swap(points);
  	int simple_count = points.size();
  	cout<<"after simplification: "<<points.size()<<endl;



*/  	

	
	return 0;
}

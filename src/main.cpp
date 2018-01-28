#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <openMVG/cameras/Camera_Common.hpp>


#include <unistd.h> 
//#include <CGAL/CORE/CORE.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>


#include <kinect.hpp>
#include <create_directory.hpp>


using namespace std;
using namespace libfreenect2;
using namespace Eigen;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::sfm;

//using namespace openMVG::geodesy;


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


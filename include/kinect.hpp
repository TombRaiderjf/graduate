#ifndef IMAGE_SAVE_PATH
#define IMAGE_SAVE_PATH
#include <Eigen/Core>



#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <map>
#include <string>
#include "create_directory.hpp"

#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/system/timer.hpp"
#include <cstdlib>
#include <stdlib.h>
#include <time.h>  
#include <stdio.h>
#include "nonFree/sift/SIFT_describer_io.hpp"




using namespace std;
using namespace libfreenect2;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::features;

class Kinect_camera{
public:
	Freenect2 freenect2;
	PacketPipeline* pipeline;
	Freenect2Device* device;
	Registration* registration;
	SyncMultiFrameListener* listener;
	//vector<Frame> rgb_array;
	//vector<Frame> depth_array;
	SfM_Data sfm_data;
	//Views & views = sfm_data.views;
  	//Intrinsics & intrinsics = sfm_data.intrinsics;
  	std::shared_ptr<IntrinsicBase> intrinsic;
  	std::unique_ptr<Image_describer> image_describer;
  	SIFT_Image_describer sift_image_describer;
  	int count;
	int flag;  // -1未创建 0未检测到设备 5打开设备失败 2打开设备 1设备处于关闭 3开始获取数据 4获取数据超时
	Kinect_camera();
	bool start();
	void close();
	bool frame_stream(bool s);
	bool save_sfm_data();
	void write_sfm_data();
	bool compute_sfm_feature(char* ImageName,/* Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& eMatrix*/ cv::Mat& rgbMat);
	bool compute_sfm_matches();
	void image_describer_out();
	void projection(cv::Mat& color, cv::Mat& depth);
};

template<class K, class V>
inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
{
    V v = defaultValue;
    typename std::map<K, V>::const_iterator i = m.find(key);
    if(i != m.end())
    {
        v = i->second;
    }
    return v;
}
#endif

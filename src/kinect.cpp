#include <kinect.hpp>
#include <keyboard_out.hpp>


using namespace std;
using namespace libfreenect2;
using namespace cv;
using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::features;
const char* img_path = "imgsave";
const string img_path_s = "imgsave";
const float focal_length = 1076;
const unsigned int img_width = 1920;
const unsigned int img_height = 1080;
const string output_path_s = "imgsave_out";
const char* output_path = "imgsave_out";

Kinect_camera::Kinect_camera(){
	flag = -1;
	count = 0;
	sfm_data.s_root_path = img_path_s; // Setup main image root_path
	if(freenect2.enumerateDevices() == 0)
  	{
    	std::cout << "no device connected!" << std::endl;
    	flag = 0;
    	return;
  	}		
  	flag = 1;
	pipeline = new CpuPacketPipeline();
	cout<<"Open default device"<<endl;
	device = freenect2.openDefaultDevice(pipeline);
	listener = new SyncMultiFrameListener(Frame::Depth|Frame::Color); //设置获取深度图和彩色图
  	if(device == 0)
  	{
   		std::cout << "failure opening device!" << std::endl;
   		flag = 5;
    	return;
  	}
	else
	{
		cout<<"Got default device!"<<endl;
		//Freenect2Device::Config config;		
		//config.EnableBilateralFilter = true;
		//config.EnableEdgeAwareFilter = true;
		//config.MinDepth = 0.3f;
		//config.MaxDepth = 4.0f;
		//device->setConfiguration(config);
		device->setColorFrameListener(listener);
		device->setIrAndDepthFrameListener(listener);
		intrinsic= std::make_shared<Pinhole_Intrinsic_Radial_K3>(img_width, img_height, focal_length, 960.237, 534.2129, 0.0796, -0.1156, 0.0067);
		image_describer.reset(new SIFT_Image_describer(SIFT_Image_describer::Params(), true));
	}
}

bool Kinect_camera::start(){
	if(!device->start()){
		return false;
	}
	flag = 2;
	cout<<"device-serial:"<<device->getSerialNumber()<<endl;
	cout<<"device firmware:"<<device->getFirmwareVersion()<<endl;
	Freenect2Device::IrCameraParams depthParams = device->getIrCameraParams();
	cout<<"Depth Camera:\n";
	cout<<"fx: "<<depthParams.fx<<"\nfy: "<<depthParams.fy<<"\ncx: "<<depthParams.cx<<"\ncy: "<<depthParams.cy<<endl;
	cout<<"k1: "<<depthParams.k1<<"\nK2: "<<depthParams.k2<<"\nk3: "<<depthParams.k3<<endl;
	Freenect2Device::ColorCameraParams colorParams = device->getColorCameraParams();	
	cout<<"Color Camera:\n";
	cout<<"fx: "<<colorParams.fx<<"\nfy: "<<colorParams.fy<<"\ncx: "<<colorParams.cx<<"\ncy: "<<colorParams.cy<<"\nshift_d: "<<colorParams.shift_d<<"\nshift_m: "<<colorParams.shift_m<<endl;
	registration = new Registration(depthParams, colorParams);
	return true;
}

void Kinect_camera::close(){
	flag = 1;
	if(device->stop())
		cout<<"success: device stopped...\n";
	else
		cout<<"error: cannot stop...\n";
	if(device->close())
		cout<<"success: device closed..."<<endl;
	else
		cout<<"error: cannot close...\n";
}

void Kinect_camera::projection(cv::Mat& color, cv::Mat& depth)
{


    Eigen::Matrix3f K_ir;           // ir内参矩阵
    //calibration
//    K_ir <<
 //       361.8275, 0, 259.9229,
 //       0, 362.7217, 209.0684,
//        0, 0, 1;
    //given 
    K_ir <<
        365.147, 0, 257.524,
        0, 365.147, 208.875,
        0, 0, 1;
    Eigen::Matrix3f K_rgb;          // rgb内参矩阵
    //calibration
//    K_rgb <<
//        1075.6, 0, 960.237,
//        0, 1077.1, 534.2129,
//        0, 0, 1;
	//given
	K_rgb<<
		1081.37,0,959.5,
		0,1081.37,539.5,
		0,0,1;
	
    Eigen::Matrix3f R_ir2rgb;
    Eigen::Matrix3f R;
    Eigen::Vector3f T_temp(54.2155,0,0);
	
	Eigen::Vector3f T;
	R = K_rgb*K_ir.inverse();
    T = K_rgb*T_temp;
    cv::Mat result(424, 512, CV_8UC3, Scalar(0,0,0));
    int i = 0;
    for (int row = 0; row < 424; row++)
    {
        for (int col = 0; col < 512; col++)
        {
            //unsigned short* p = (unsigned short*)depth.data;
            //unsigned short depthValue = p[row * 512 + col];
            float depthValue = depth.at<float>(row, col);
            //cout << "depthValue       " << depthValue << endl;
            if (!(isnan(depthValue) || depthValue <= 0.001))
            {
                // 投影到彩色图上的坐标
                Eigen::Vector3f uv_depth(col, row, 1.0f);
                Eigen::Vector3f uv_color = depthValue / 1000.f*R*uv_depth + T / 1000;   //用于计算映射，核心式子

                int X = static_cast<int>(uv_color[0] / uv_color[2]);         //计算X，即对应的X值
                int Y = static_cast<int>(uv_color[1] / uv_color[2]);         //计算Y，即对应的Y值
				
                if ((X >= 0 && X < 1920) && (Y >= 0 && Y < 1080))
                {

                    result.data[i * 3] = color.at<Vec4b>(Y,X)[0];
                    result.data[i * 3 + 1] = color.at<Vec4b>(Y,X)[1];
                    result.data[i * 3 + 2] = color.at<Vec4b>(Y,X)[2];
                }
            }
            i++;
        }
    }
    cv::imshow("projection", result);
    cv::waitKey();

}



void mergeImg(cv::Mat & dst, cv::Mat &src1, cv::Mat &src2)  
{  
    int rows = src1.rows+5+src2.rows;  
    int cols = src1.cols+5+src2.cols;  
    CV_Assert(src1.type () == src2.type ());  
    dst.create (rows,cols,src1.type ());  
    src1.copyTo (dst(Rect(0,0,src1.cols,src1.rows)));  
    src2.copyTo (dst(Rect(src1.cols+5,0,src2.cols,src2.rows)));  
} 

bool Kinect_camera::frame_stream(bool s){
	flag = 3;
	Frame undistorted(512, 424, 4), registered(512, 424, 4), bigdepth(1920, 1082, 4);
	int pre_count = 0;
	
	
	//生成去畸变参数
	
	
	        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    		cameraMatrix.at<double>(0, 0) = 1075.6;
    		cameraMatrix.at<double>(0, 1) = 0;
    		cameraMatrix.at<double>(0, 2) = 960.237;
    		cameraMatrix.at<double>(1, 1) = 1077.1;
    		cameraMatrix.at<double>(1, 2) = 534.2129;

    		cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    		distCoeffs.at<double>(0, 0) = 0.0796;
    		distCoeffs.at<double>(1, 0) = -0.1156;
    		distCoeffs.at<double>(2, 0) = 0;
    		distCoeffs.at<double>(3, 0) = 0;
    		distCoeffs.at<double>(4, 0) = 0.0067;
            
            cv::Mat map1, map2;
    		Size imageSize(1920,1080);
            cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
        
        	
	
	while(/*!kbhit()*/pre_count<20){
	
		clock_t m1=clock();
		FrameMap frames;
		
		if (!listener->waitForNewFrame(frames, 10*1000)) // 10 seconds
    	{
    		std::cout << "timeout!" << std::endl;
    		flag = 4;
     		return false;
   		}
    	Frame *rgb = frames[libfreenect2::Frame::Color];
    	Frame *depth = frames[libfreenect2::Frame::Depth];
    	
    	
		//registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth, 0);
		registration->apply(rgb, depth, &undistorted, &registered);
		

        if(rgb && depth)
        {
        
            cv::Mat depthMat((int)undistorted.height, (int)undistorted.width, CV_32FC1, undistorted.data);
            cv::Mat rgbMat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data);
                
            cv::Mat rgb_cal;
            remap(rgbMat, rgb_cal, map1, map2, INTER_LINEAR);           
            
            //cv::Mat outImg;  
    		//mergeImg (outImg,rgbMat,rgb_cal);  
    		//imshow("img",outImg);  
    		//waitKey(); 
			
            cv::Mat regMat((int)registered.height, (int)registered.width, CV_8UC4, registered.data);
            //cv::Mat bigdepthMat((int)bigdepth.height, (int)bigdepth.width, CV_32FC1, bigdepth.data);
            
            
            char ImageName[20];
            sprintf(ImageName, "%s%s%d%s", img_path, "/img", count, ".jpg");
            imwrite(ImageName, rgbMat);   
            //imwrite(ImageName, regMat); 
            
            write_sfm_data();
            
            //cv::Mat bigdepthMat_8u(bigdepthMat.rows, bigdepthMat.cols, CV_8UC4, bigdepthMat.data);     
            cv::Mat depthMat_8u(depthMat.rows, depthMat.cols, CV_8UC4, depthMat.data);    
                    
            char ImageName_reg[20];
            sprintf(ImageName_reg, "%s%s%d%s", img_path, "/img", count, ".bmp");  
            //imwrite(ImageName_reg, bigdepthMat_8u);
            imwrite(ImageName_reg, depthMat_8u);
            		
		
            if( compute_sfm_feature(ImageName, rgbMat) )
            {
            	std::cout<<"success: extract feature\n";
            }
            else
            {
            	std::cout<<"error: extract feature\n";
            	return false;
            }

            count++;
            cv::imshow("registered", rgb_cal);
            cv::waitKey();
            
            //projection(rgbMat, depthMat);

		}
		
		listener->release(frames);
		clock_t m2=clock();
		std::cout<<"Single Frame time:"<<double(m2-m1)/CLOCKS_PER_SEC<<endl;
		pre_count++;
        
	}
}


void Kinect_camera::write_sfm_data()
{
	//string path = img_path_s + "/img" + to_string(count) + ".jpg";
	string path = "img" + to_string(count) + ".jpg";
	View v(path, sfm_data.views.size(), sfm_data.views.size(), sfm_data.views.size(), img_width, img_height);
	sfm_data.intrinsics[v.id_intrinsic] = intrinsic;
	sfm_data.views[v.id_view] = std::make_shared<View>(v);
}

bool Kinect_camera::compute_sfm_feature(char* ImageName, cv::Mat& rgbMat){
	
	std::string sOutDir= output_path_s+"/matches";
	clock_t c1 = clock();
	//Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eMatrix;
	
	cv::Mat gray;
	cvtColor(rgbMat, gray, CV_RGBA2GRAY);
	//imshow("v",gray);
	//waitKey();
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B;
	std::cout<<"graygraygray-----------\n";
	cv::cv2eigen(gray, B);

// Map the OpenCV matrix with Eigen:
	//Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A_Eigen(gray.ptr<unsigned char>(), gray.rows, gray.cols);

// Do something with it in Eigen, create e.g. a new Eigen matrix:
	//Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> B = A_Eigen.inverse();

// create an OpenCV Mat header for the Eigen data:
			//cv::Mat B_OpenCV(B.rows(), B.cols(), CV_32FC1, B.data());
	std::cout<<"suceess!--------\n";
	
	Image<unsigned char> imageGray(B);	
	
	/*if (!ReadImage(ImageName, &imageGray))
	{
		std::cout<<"width:"<<w<<"\nheight:"<<h<<"\ndepth:"<<d<<endl;
        return false;
    }
    */
    clock_t c2 = clock();

    auto regions = sift_image_describer.DescribeSIFT(imageGray);
    
	const std::string
        sView_filename = stlplus::create_filespec(sfm_data.s_root_path, sfm_data.views[count]->s_Img_path),
        sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
        sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");
	if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
          std::cerr << "Cannot save regions for images: " << sView_filename << std::endl
                    << "Stopping feature extraction." << std::endl;
          return false;
    }
    
    std::cout<<"compute feature time:"<<double(c2-c1)/CLOCKS_PER_SEC<<endl;	
    return true;
}


void Kinect_camera::image_describer_out()
{
	std::string sOutDir= output_path_s+"/matches";
	const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");
    std::ofstream stream(sImage_describer.c_str());
    cereal::JSONOutputArchive archive(stream);
    archive(cereal::make_nvp("image_describer", image_describer));
    auto regionsType = image_describer->Allocate();
    archive(cereal::make_nvp("regions_type", regionsType));
}

bool Kinect_camera::compute_sfm_matches()
{
	return true;
}

bool Kinect_camera::save_sfm_data()
{
	GroupSharedIntrinsics(sfm_data);
	std::string sOutDir= output_path_s+"/matches";
	if (Save(sfm_data,stlplus::create_filespec( sOutDir, "sfm_data.json" ).c_str(), ESfM_Data(VIEWS|INTRINSICS)))
	{	
		std::cout<<"success: create sfm_data.json\n";
		return true;
	}
	else
	{
		std::cerr << "failed: can not create sfm_data_json\n";
		return false;
	}
}



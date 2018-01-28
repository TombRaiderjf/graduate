#include <create_directory.hpp>

using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;


bool create_directory(){   
    if(stlplus::is_folder (output_path_s))
    {
    	printf("path already exist: %s\n", output_path);
    	if( stlplus::folder_delete ( output_path_s, true))
    	{
    		printf("path delete: %s\n", output_path);
    	}
    	else
    	{
    		printf("delete error!\n");
    		return false;
    	}
    }
    if(stlplus::folder_create (output_path_s))
    {
    	printf("create path: %s\n", output_path);
    }
    else
    {
    	printf("create path failed!: %s\n", output_path);
    	return false;
    }
    std::string matches = output_path_s + "/matches";
    std::string reconstruction = output_path_s + "/reconstruction";
    if(stlplus::folder_create (matches) && stlplus::folder_create (reconstruction))
    {
    	printf("create path: %s, %s\n", matches.c_str(), reconstruction.c_str());
    }
    else
    {
    	printf("create path failed!: %s, %s\n", matches.c_str(), reconstruction.c_str());
    	return false;
    }
    
    if(stlplus::is_folder (img_path_s))
    {
    	stlplus::folder_delete ( img_path_s, true);
    	stlplus::folder_create ( img_path_s );
    }

   	return true;
}



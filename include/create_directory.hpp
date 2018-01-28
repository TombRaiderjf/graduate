#ifndef SFM_DATA_WRITE
#define SFM_DATA_WRITE

#include "openMVG/cameras/cameras.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"

#include "openMVG/image/image_io.hpp"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/types.hpp"
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>

#include "openMVG/third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include <define.hpp>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <string>
#include <iostream>


bool create_directory();

#endif

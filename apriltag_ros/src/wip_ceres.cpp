#include "apriltag_ros/ceres_bundle_solver.hpp"

using namespace apriltag_ros;




int main(int argc, char **argv) {

    CeresBundleSolver s;

    std::vector<calibration_snapshot> data;

    std::map<int, raw_pose> tag_poses;
    std::map<std::string, raw_pose> cam_poses;

    tag_poses[1].translation = {2, 1, 1};
    tag_poses[1].quaternion = {0, 0, 0, 1};

    // tag_poses[1].translation = {2, 1, 1};
    // tag_poses[1].quaternion = {0, 0, 0, 1};

    cam_poses["cam_0"].translation = {-2, -1.5, 0.5};
    cam_poses["cam_0"].quaternion = {0, 0, 0, 1};

    tag_correspondence tag;
    tag.id = 1;
    tag.im_corners = std::array<std::array<double,2 >, 4>{{
        {5,5},
        {5,-5},
        {-5, -5},
        {-5, 5}}
    };

    tag.obj_corners = std::array<std::array<double, 2>, 4>{{
        {1, 1},
        {1, -1},
        {-1, -1},
        {-1, 1}}
    };


    calibration_snapshot datum;
    datum.tags.push_back(tag);

    datum.camera_name = "cam_0";
    // datum.camera_info->K[0] = 10;
    // datum.camera_info->K[4] = 10;
    // datum.camera_info->K[8] = 1;
    
    data.push_back(datum);

    
    
    s.solve(data, tag_poses, cam_poses, 1);




    
}



#include "apriltag_ros/ceres_bundle_solver.hpp"


using namespace apriltag_ros;
using namespace ceres;

double CeresBundleSolver::solve(const std::vector<calibration_snapshot> &data, std::map<int, raw_pose> &tag_poses,
                                std::map<std::string, raw_pose> &cam_poses, int fixed_tag_id)
{

    // Build the problem.
    Problem problem;

    std::cout << "setting up problem\n";
    ceres::LocalParameterization* quaternion_local_parameterization =
        new EigenQuaternionParameterization;

    for(const auto &datum: data)
    {

        std::cout << "setting up tags for camera " << datum.camera_name << "\n";
        image_geometry::PinholeCameraModel camera_model;
        camera_model.fromCameraInfo(datum.camera_info);
        double fx = camera_model.fx(); // focal length in camera x-direction [px]
        double fy = camera_model.fy(); // focal length in camera y-direction [px]
        double cx = camera_model.cx(); // optical center x-coordinate [px]
        double cy = camera_model.cy(); // optical center y-coordinate [px]

        // double fx=10;
        // double fy=10;
        // double cx=0;
        // double cy=0;

        // std::cout << "fx: " << fx << ", fy: " << fy << ", cx: " << cx << ", cy: " << cy << "\n";


        raw_pose &cam_pose = cam_poses[datum.camera_name];


        for(const auto &tag: datum.tags)
        {
            raw_pose &tag_pose = tag_poses[tag.id];

            std::cout << "Adding tag " << tag.id << "\n";
            for(int c=0; c < 4; c++)
            {
                CostFunction* camera_cost_function_1 =
                    new AutoDiffCostFunction<ReprojectionCostFunctor, 2, 3, 4, 3, 4>(
                        new ReprojectionCostFunctor(tag.im_corners[c][0], tag.im_corners[c][1],
                                                    tag.obj_corners[c][0], tag.obj_corners[c][1],
                                                    fx, fy, cx, cy));

                
                problem.AddResidualBlock(camera_cost_function_1, NULL,
                                         tag_pose.translation.data(), tag_pose.quaternion.data(),
                                         cam_pose.translation.data(), cam_pose.quaternion.data());
            }
            problem.SetParameterization(tag_pose.quaternion.data(), quaternion_local_parameterization);
        }

        problem.SetParameterization(cam_pose.quaternion.data(), quaternion_local_parameterization);
    }


    //TODO: HARD CODED base position of tag 1
    problem.SetParameterBlockConstant(tag_poses[fixed_tag_id].translation.data());
    problem.SetParameterBlockConstant(tag_poses[fixed_tag_id].quaternion.data());

    // double cost = 0.0;
    // problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    // std::cout << "Initial problem.Evaluate: " << cost << "\n";
    
    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    // std::cout << summary.FullReport() << "\n";
    
    for(const auto &tag: tag_poses)
    {
        std::cout << "tag " << tag.first << ": " << tag.second << "\n";
    }
    for(const auto &cam: cam_poses)
    {
        std::cout << "cam " << cam.first << ": " << cam.second << "\n";
    }


    
    return -1;
}

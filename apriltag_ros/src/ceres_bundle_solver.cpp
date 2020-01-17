#include "apriltag_ros/ceres_bundle_solver.hpp"


using namespace apriltag_ros;
using namespace ceres;

double CeresBundleSolver::solve(const std::vector<calibration_datum> &data, std::map<int, raw_pose> &tag_poses,
             std::map<std::string, raw_pose> &cam_poses)
{

    // Build the problem.
    Problem problem;




    // std::vector<double> p_tag{2, 1, 1};
    // std::vector<double> q_tag{0,0,0,1};
    // std::vector<double> p_cam{0,0,0};
    // std::vector<double> q_cam{0,0,0,1};


    // std::vector<std::vector<double> > im{
    //     {5,5},
    //     {5,-5},
    //     {-5, -5},
    //     {-5, 5}
    // };

    // std::vector<std::vector<double> > obj{
    //     {1, 1},
    //     {1, -1},
    //     {-1, -1},
    //     {-1, 1}
    // };
    // double im_x = 5;
    // double im_y = 5;

    // double obj_x = 1;
    // double obj_y = 1;

    // std::cout << "images point is at: " << im_x << ", " << im_y << "\n";
    // std::cout << "object point is at: " << obj_x << ", " << obj_y << "\n";


    // auto a = CameraCostFunctor(im_x, im_y, obj_x, obj_y, fx, fy, cx, cy);
    // double residual[2];
    // a(p_tag.data(), q_tag.data(), residual);

    // return 0;
    

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

        std::cout << "fx: " << fx << ", fy: " << fy << ", cx: " << cx << ", cy: " << cy << "\n";


        raw_pose &cam_pose = cam_poses[datum.camera_name];


        for(const auto &tag: datum.tags)
        {
            raw_pose &tag_pose = tag_poses[tag.id];

            // if(tag.id != 1)
            // {
            //     continue;
            // }
            
            for(int c=0; c < 4; c++)
            {
                CostFunction* camera_cost_function_1 =
                    new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4, 3, 4>(
                        new CameraCostFunctor(tag.im_corners[c][0], tag.im_corners[c][1],
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

        // CostFunction* camera_cost_function_2 =
        //     new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
        //         new CameraCostFunctor(im[1][0], im[1][1], obj[c][0], obj[1][1], fx, fy, cx, cy));
        // problem.AddResidualBlock(camera_cost_function_2, NULL, p_tag.data(), q_tag.data());

        // CostFunction* camera_cost_function_3 =
        //     new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
        //         new CameraCostFunctor(im[2][0], im[2][1], obj[c][0], obj[][1], fx, fy, cx, cy));
        // problem.AddResidualBlock(camera_cost_function_3, NULL, p_tag.data(), q_tag.data());

        // CostFunction* camera_cost_function_4 =
        //     new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
        //         new CameraCostFunctor(im[3][0], im[3][1], obj[c][0], obj[c][1], fx, fy, cx, cy));
        // problem.AddResidualBlock(camera_cost_function_4, NULL, p_tag.data(), q_tag.data());
    // }

    // problem.SetParameterization(q_tag.data(), quaternion_local_parameterization);
    // problem.SetParameterization(q_cam.data(), quaternion_local_parameterization);


    // problem.SetParameterBlockConstant(p_tag.data());
    // problem.SetParameterBlockConstant(q_tag.data());




    //TODO: HARD CODED
    problem.SetParameterBlockConstant(tag_poses[1].translation.data());
    problem.SetParameterBlockConstant(tag_poses[1].quaternion.data());


    
    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
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

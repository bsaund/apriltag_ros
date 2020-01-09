#include "apriltag_ros/ceres_bundle_solver.hpp"

using namespace apriltag_ros;
using namespace ceres;

double CeresBundleSolver::solve(double x)
{
    // google::InitGoogleLogging(argv[0]);

    // The variable to solve for with its initial value.
    double initial_x = x;
    // double x = initial_x;

    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    // CostFunction* cost_function =
    //     new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    // problem.AddResidualBlock(cost_function, NULL, &x);



    std::vector<double> p_tag{0, 0, 1.8};
    std::vector<double> q_tag{0,0,0,1};

    double fx = 10;
    double fy = 10;
    double cx = 0;
    double cy = 0;

    double im_x = 10;
    double im_y = 10;

    double obj_x = 2;
    double obj_y = 2;

    std::cout << "images point is at: " << im_x << ", " << im_y << "\n";
    std::cout << "object point is at: " << obj_x << ", " << obj_y << "\n";
    

    ceres::LocalParameterization* quaternion_local_parameterization =
        new EigenQuaternionParameterization;
    
    // {
        CostFunction* camera_cost_function_1 =
            new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
                new CameraCostFunctor(im_x, im_y, obj_x, obj_y, fx, fy, cx, cy));
        problem.AddResidualBlock(camera_cost_function_1, NULL, p_tag.data(), q_tag.data());

        // CostFunction* camera_cost_function_2 =
        //     new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
        //         new CameraCostFunctor(10, -10, 1, -1, fx, fy, cx, cy));
        // problem.AddResidualBlock(camera_cost_function_2, NULL, p_tag.data(), q_tag.data());

        // CostFunction* camera_cost_function_3 =
        //     new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
        //         new CameraCostFunctor(-10, -10, -1, -1, fx, fy, cx, cy));
        // problem.AddResidualBlock(camera_cost_function_3, NULL, p_tag.data(), q_tag.data());

        // CostFunction* camera_cost_function_4 =
        //     new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
        //         new CameraCostFunctor(-10, 10, -1, 1, fx, fy, cx, cy));
        // problem.AddResidualBlock(camera_cost_function_4, NULL, p_tag.data(), q_tag.data());
    // }

    problem.SetParameterization(q_tag.data(), quaternion_local_parameterization);








    
    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    // std::cout << summary.FullReport() << "\n";

    std::cout << "final tag position: pos: " << p_tag[0] << ", " << p_tag[1] << ", " << p_tag[2] <<
        "  rot: " << q_tag[0] << ", " << q_tag[1] << ", " << q_tag[2] << ", " << q_tag[3] << "\n";
    

    return -1;
}

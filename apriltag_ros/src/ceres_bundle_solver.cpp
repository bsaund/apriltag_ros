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



    std::vector<double> p_tag{2, 1, 1};
    std::vector<double> q_tag{0,0,0,1};

    double fx = 10;
    double fy = 10;
    double cx = 0;
    double cy = 0;

    std::vector<std::vector<double> > im{
        {5,5},
        {5,-5},
        {-5, -5},
        {-5, 5}
    };

    std::vector<std::vector<double> > obj{
        {1, 1},
        {1, -1},
        {-1, -1},
        {-1, 1}
    };
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
    

    ceres::LocalParameterization* quaternion_local_parameterization =
        new EigenQuaternionParameterization;
    
    {
        CostFunction* camera_cost_function_1 =
            new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
                new CameraCostFunctor(im[0][0], im[0][1], obj[0][1], obj[1][0], fx, fy, cx, cy));
        problem.AddResidualBlock(camera_cost_function_1, NULL, p_tag.data(), q_tag.data());

        CostFunction* camera_cost_function_2 =
            new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
                new CameraCostFunctor(im[1][0], im[1][1], obj[1][0], obj[1][1], fx, fy, cx, cy));
        problem.AddResidualBlock(camera_cost_function_2, NULL, p_tag.data(), q_tag.data());

        CostFunction* camera_cost_function_3 =
            new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
                new CameraCostFunctor(im[2][0], im[2][1], obj[2][0], obj[2][1], fx, fy, cx, cy));
        problem.AddResidualBlock(camera_cost_function_3, NULL, p_tag.data(), q_tag.data());

        CostFunction* camera_cost_function_4 =
            new AutoDiffCostFunction<CameraCostFunctor, 2, 3, 4>(
                new CameraCostFunctor(im[3][0], im[3][1], obj[3][0], obj[3][1], fx, fy, cx, cy));
        problem.AddResidualBlock(camera_cost_function_4, NULL, p_tag.data(), q_tag.data());
    }

    problem.SetParameterization(q_tag.data(), quaternion_local_parameterization);


    // problem.SetParameterBlockConstant(q_tag.data());






    
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

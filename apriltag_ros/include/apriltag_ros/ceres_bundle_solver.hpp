#ifndef APRILTAG_ROS_CERES_BUNDLE_SOLVER
#define APRILTAG_ROS_CERES_BUNDLE_SOLVER

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace apriltag_ros
{
    
    // struct CostFunctor {
    //     template <typename T>
    //     bool operator()(const T* const x, T* residual) const {
    //         residual[0] = T(10.0) - x[0];
    //         return true;
    //     }
    // };

    struct CameraCostFunctor {
        double im_x, im_y;
        double obj_x, obj_y;
        double fx, fy, cx, cy;
        Eigen::Matrix<double, 3, 4> camera_intrinsics;
        // Eigen::Matrix<double, 4, 1> object_point;
        // Eigen::Isometry3d object_point;
        
        CameraCostFunctor(double im_x_, double im_y_, double obj_x_, double obj_y_,
                          double fx_, double fy_, double cx_, double cy_) :
            im_x(im_x_), im_y(im_y_), obj_x(obj_x_), obj_y(obj_y_), fx(fx_), fy(fy_), cx(cx_), cy(cy_)
        {
        }

        
        template <typename T>
        bool operator()(const T* const p_tag_ptr, const T* const q_tag_ptr,
                        // const T* const p_cam_ptr, const T* const q_cam_ptr, 
                        T* residual) const {
            // residual[0] = T(10.0) - x[0];
            Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_tag(p_tag_ptr);
            Eigen::Map<const Eigen::Quaternion<T> >   q_tag(q_tag_ptr);
            // Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_cam(p_cam_ptr);
            // Eigen::Map<const Eigen::Quaternion<T> >   q_cam(q_cam_ptr);

            // Eigen::Translation3d v;
            // v.x() = 0;
            // v.y() = 0;
            // v.z() = 0;
            // Eigen::Translation3d v1 = q_tag*v;
            // Eigen::Isometry3d tag(p_tag, q_tag);
            // std::cout << "q_tag rotation: " << q_tag.toRotationMatrix() << "\n";
            // q_tag.toRotationMatrix();
            // p_tag * q_tag * object_point;
            // q_tag * object_point;

            Eigen::Matrix<T, 4, 1> object_point;
            object_point << T(obj_x),
                T(obj_y),
                T(0),
                T(1);

            Eigen::Matrix<T, 3, 4> extrinsic;
            extrinsic << q_tag.toRotationMatrix(), p_tag;

            Eigen::Matrix<T, 3, 3> intrinsic;
            intrinsic(0,0) = T(fx);
            intrinsic(1,1) = T(fy);
            intrinsic(0,2) = T(cx);
            intrinsic(1,2) = T(cy);
            intrinsic(2,2) = T(1);

            Eigen::Matrix<T, 3, 1> image_output = intrinsic * extrinsic * object_point;
            
            
            // residual[0] = image_output(0,0) - T(im_x);
            // residual[1] = image_output(1,0) - T(im_y);
            // residual[0] = image_output(0,0)/image_output(2,0) - T(im_x);
            // residual[1] = image_output(1,0)/image_output(2,0) - T(im_y);
            residual[0] = T(im_x) - extrinsic(0,3);
            residual[1] = T(im_y) - extrinsic(1,3);
            return true;
        }
        
    };
        


    class CeresBundleSolver
    {
    public:
        double solve(double x);
    };
}

#endif //APRILTAG_ROS_CERES_BUNDLE_SOLVER

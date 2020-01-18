#ifndef APRILTAG_ROS_CERES_BUNDLE_SOLVER
#define APRILTAG_ROS_CERES_BUNDLE_SOLVER

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "apriltag_ros/calibration_types.h"

namespace apriltag_ros
{
    //Residual for a single object <-> image point correspondence
    struct CameraCostFunctor {
        double im_x, im_y;
        double obj_x, obj_y;
        double fx, fy, cx, cy;
        
        CameraCostFunctor(double im_x_, double im_y_, double obj_x_, double obj_y_,
                          double fx_, double fy_, double cx_, double cy_) :
            im_x(im_x_), im_y(im_y_), obj_x(obj_x_), obj_y(obj_y_), fx(fx_), fy(fy_), cx(cx_), cy(cy_)
        {
        }

        
        template <typename T>
        bool operator()(const T* const p_tag_ptr, const T* const q_tag_ptr,
                        const T* const p_cam_ptr, const T* const q_cam_ptr, 
                        T* residual) const {
            // residual[0] = T(10.0) - x[0];
            Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_tag(p_tag_ptr);
            Eigen::Map<const Eigen::Quaternion<T> >   q_tag(q_tag_ptr);
            Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_cam(p_cam_ptr);
            Eigen::Map<const Eigen::Quaternion<T> >   q_cam(q_cam_ptr);

            Eigen::Matrix<T, 3, 1> object_point;
            object_point << T(obj_x),
                T(obj_y),
                T(0);

            //Transform object point to image frame
            object_point = q_tag * object_point + p_tag;
            object_point = q_cam * object_point + p_cam;

            
            Eigen::Matrix<T, 3, 3> intrinsic;
            intrinsic(0,0) = T(fx);
            intrinsic(1,1) = T(fy);
            intrinsic(0,2) = T(cx);
            intrinsic(1,2) = T(cy);
            intrinsic(2,2) = T(1);

            Eigen::Matrix<T, 3, 1> image_output = intrinsic * object_point;
            
            
            residual[0] = image_output(0,0)/image_output(2,0) - T(im_x);
            residual[1] = image_output(1,0)/image_output(2,0) - T(im_y);
            return true;
        }
        
    };

    
    class CeresBundleSolver
    {
    public:
        double solve(const std::vector<calibration_datum> &data, std::map<int, raw_pose> &tag_poses,
                     std::map<std::string, raw_pose> &cam_poses, int fixed_tag_id);
    };
}

#endif //APRILTAG_ROS_CERES_BUNDLE_SOLVER

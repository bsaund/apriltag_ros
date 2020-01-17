#ifndef APRILTAG_ROS_CALIBRATION_TYPES
#define APRILTAG_ROS_CALIBRATION_TYPES

#include "apriltag_ros/common_functions.h"
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

namespace apriltag_ros {

    struct raw_pose
    {
        std::vector<double> translation;
        std::vector<double> quaternion; //in the form {x, y, z, w}

    raw_pose():
        translation{0,0,0},
            quaternion{0,0,0,1}
        {}

    raw_pose(geometry_msgs::TransformStamped tf):
        translation{tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z},
            quaternion{tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w}
        {
        }

        geometry_msgs::Transform toRosTransformMsg()
        {
            geometry_msgs::Transform tf;
            tf.translation.x = translation[0];
            tf.translation.y = translation[1];
            tf.translation.z = translation[2];
            tf.rotation.x = quaternion[0];
            tf.rotation.y = quaternion[1];
            tf.rotation.z = quaternion[2];
            tf.rotation.w = quaternion[3];
            return tf;
        }
        
        friend std::ostream& operator<<(std::ostream& os, const raw_pose &p);

    };
    

    

    /*
     *  Custom structs for storing tag detection data
     *  These are easier to work with than zarrays defined in 
     *  the apriltag library
     */
    struct tag_for_calibration
    {
        int id;

        std::array<std::array<double, 2>, 4> im_corners;
        std::array<std::array<double, 2>, 4> obj_corners;

        tag_for_calibration(const apriltag_detection_t* original);
        tag_for_calibration() = default;
    };
    
    struct calibration_datum
    {
        std::string camera_name;
        std::vector<tag_for_calibration> tags;
        sensor_msgs::CameraInfoConstPtr camera_info;
        calibration_datum(const zarray_t* detections);
        calibration_datum() = default;
    };
}


#endif //APRILTAG_ROS_CALIBRATION_TYPES

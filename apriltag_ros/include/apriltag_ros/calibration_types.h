#ifndef APRILTAG_ROS_CALIBRATION_TYPES
#define APRILTAG_ROS_CALIBRATION_TYPES

#include "apriltag_ros/common_functions.h"
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include "common/homography.h"


/*****************************************************************************
 ** This is a header-only definition of data structures
 ** used in bundle calibration
 *****************************************************************************/

namespace apriltag_ros {

    /*
     *  Pose stored as two std::vectors for translation and rotation (quaternion)
     */
    struct raw_pose
    {
        std::vector<double> translation;
        std::vector<double> quaternion; //in the form {x, y, z, w}

    raw_pose():
        translation{0,0,0},
        quaternion{0,0,0,1}
        {}

    raw_pose(geometry_msgs::TransformStamped tf) :
        translation{tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z},
        quaternion{tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w}
        {}

    geometry_msgs::Transform toRosTransformMsg() const
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
    

    

    /*****************************************************
     *  Custom structs for storing tag detection data
     *  These are easier to work with than zarrays defined in 
     *  the apriltag library
     *****************************************************/

    /**
     *  tag_correspondence stores the 4 corners of a tag as object points (in 3D with z=0) 
     *  and the corresponding corners as observed in the image 
     */
    struct tag_correspondence
    {
    public:
        int id;

        std::array<std::array<double, 2>, 4> im_corners;
        std::array<std::array<double, 2>, 4> obj_corners;

    public:
        tag_correspondence() = default;

        tag_correspondence(const apriltag_detection_t* original, double tag_size)
        : id(original->id)
        {
            // Add to image point vector the tag corners in the image frame
            // Going counterclockwise starting from the bottom left corner
            double tag_x[4] = {-1,1,1,-1};
            double tag_y[4] = {1,1,-1,-1}; // Negated because AprilTag tag local
            // frame has y-axis pointing DOWN
            // while we use the tag local frame
            // with y-axis pointing UP
            for(int i=0; i<4; i++)
            {
                double im_x, im_y;
                homography_project(original->H, tag_x[i], tag_y[i], &im_x, &im_y);
                im_corners[i][0] = im_x;
                im_corners[i][1] = im_y;
            }


            // Add to object point vector the tag corner coordinates in the bundle frame
            // Going counterclockwise starting from the bottom left corner

            obj_corners = std::array<std::array<double, 2>, 4>{{
                    {-tag_size, -tag_size},
                    { tag_size, -tag_size},
                    { tag_size,  tag_size},
                    {-tag_size,  tag_size}}
            };

        }

    };

    /**
     *   calibration_snapshot stores the tag correspondences observed
     *   in a single camera frame
     */
    struct calibration_snapshot
    {
        std::string camera_name;
        std::vector<tag_correspondence> tags;
        sensor_msgs::CameraInfoConstPtr camera_info;
        calibration_snapshot() = default;

        calibration_snapshot(const zarray_t* detections, std::map<int, double> tag_sizes)
        {
            for(int i=0; i<zarray_size(detections); i++)
            {
                apriltag_detection_t* detection;
                zarray_get(detections, i, &detection);
                tags.emplace_back(detection, tag_sizes[detection->id]);  //TODO: hard coded tag size
            }

            std::sort(tags.begin(), tags.end(),
                      [](tag_correspondence &lhs, tag_correspondence &rhs) {return lhs.id < rhs.id;});
        }
        
    };

    inline std::ostream& operator<<(std::ostream& os, const raw_pose &p)
    {
        os <<
            "x: " << p.translation[0] << ", " <<
            "y: " << p.translation[1] << ", " <<
            "z: " << p.translation[2] << ", " <<
            "qx: " << p.quaternion[0] << ", " <<
            "qy: " << p.quaternion[1] << ", " <<
            "qz: " << p.quaternion[2] << ", " <<
            "qw: " << p.quaternion[3];
        return os;
    }

}


#endif //APRILTAG_ROS_CALIBRATION_TYPES

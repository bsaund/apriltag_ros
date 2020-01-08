#ifndef APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_
#define APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QPixmap>
#include "apriltag_ros/common_functions.h"


#define PIXEL_MOTION_THRESHOLD_FOR_NEW_CALIBRATION_POINT 30.0


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace apriltag_ros {

/*****************************************************************************
 ** Class
 *****************************************************************************/

    struct tag_for_calibration
    {
        int id;
        std::array<std::array<double, 2>, 4> corners;
        matd_t* H;


        tag_for_calibration(int id_, const double p[4][2], const matd_t* H_):
            id(id_)
        {
            for(int i=0; i < 4; i++)
            {
                corners[i][0] = p[i][0];
                corners[i][1] = p[i][1];
            }
            H = matd_copy(H_);
        }

        tag_for_calibration(const apriltag_detection_t* orig):
            tag_for_calibration(orig->id, orig->p, orig->H)
        {            
        }
    };
    
    struct calibration_datum
    {
        std::vector<tag_for_calibration> tags;

        calibration_datum(const zarray_t* detections)
        {
            for(int i=0; i<zarray_size(detections); i++)
            {
                apriltag_detection_t* detection;
                zarray_get(detections, i, &detection);
                tags.emplace_back(detection);
            }

            std::sort(tags.begin(), tags.end(),
                      [](tag_for_calibration &lhs, tag_for_calibration &rhs) {return lhs.id < rhs.id;});
        }
    };
    

    class QNode : public QThread {
        Q_OBJECT
    public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
        QPixmap& getPixmap();
        std::set<int> visible_tags;

        std::vector<calibration_datum> calibration_data;

    public:
        const std::vector<TagBundleDescription>& getTagBundleDescriptions()
        {
            return tag_detector_->getTagBundleDescriptions();
        }



    Q_SIGNALS:
        void imageUpdated();
        void newTagObserved(int id);
        void rosShutdown();

    protected:
        int init_argc;
        char** init_argv;

        std::unique_ptr<image_transport::ImageTransport> it_;
        image_transport::CameraSubscriber camera_image_subscriber_;
        std::unique_ptr<TagDetector> tag_detector_;
        cv_bridge::CvImagePtr cv_image_;
        QPixmap pixmap;
        std::set<int> observed_tags;

    protected:
        void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                           const sensor_msgs::CameraInfoConstPtr& camera_info);
        
        void addToObservedSet(int id);
        bool tooSimilarToPrevious(const calibration_datum &cur) const;
            

    };

}  // namespace apriltag_ros

#endif /* APRILTAG_ROS_BUNDLE_CALIBRATION_QT_ROS_QNODE_HPP_ */

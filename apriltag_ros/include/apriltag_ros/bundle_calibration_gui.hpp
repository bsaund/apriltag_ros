#ifndef APRILTAG_ROS_BUNDLE_CALIBRATION_GUI
#define APRILTAG_ROS_BUNDLE_CALIBRATION_GUI

#include <QtWidgets/QMainWindow>
#include <QThread>
#include <QStringListModel>

#include "ui_bundle_calibration.h"

#include "apriltag_ros/bundle_calibration_qt_ros.hpp"


namespace apriltag_ros
{
    /*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
    class MainWindow : public QMainWindow {
        Q_OBJECT

    public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


    public Q_SLOTS:
    //     /******************************************
    //      ** Auto-connections (connectSlotsByName())
    //      *******************************************/
        void on_button_start_calibration_clicked(bool check);

    //     void on_actionAbout_triggered();
    //     void on_button_connect_clicked(bool check );
    //     void on_checkbox_use_environment_stateChanged(int state);

    //     /******************************************
    //      ** Manual connections
    //      *******************************************/
        void updateDisplayedImage();
        void addTagToChecklist(int id);

    protected:
        void setupBundleCheckboxes();
        
    protected:
	Ui::ApriltagBundleCalibration ui;
	QNode qnode;
        int checkbox_offset = 35;
        QButtonGroup checkboxes;
        std::map<int, QLabel> specified_tags;
    };

}

#endif // APRILTAG_ROS_BUNDLE_CALIBRATION_GUI

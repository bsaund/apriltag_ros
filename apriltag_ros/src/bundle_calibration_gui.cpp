#include "apriltag_ros/bundle_calibration_gui.hpp"



#include <QApplication>
#include <QtWidgets>
#include <iostream>


using namespace apriltag_ros;

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      qnode(argc, argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    // ReadSettings();
    // setWindowIcon(QIcon(":/images/icon.png"));
    // ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(imageUpdated()), this, SLOT(updateDisplayedImage()));

    /*********************
     ** Logging
     **********************/
    // ui.view_logging->setModel(qnode.loggingModel());
    // QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
     ** Auto Start
     **********************/
    // if ( ui.checkbox_remember_settings->isChecked() ) {
    //     on_button_connect_clicked(true);
    // }
    qnode.init();
}

void MainWindow::updateDisplayedImage()
{
    std::cout << "Updating displayed image\n";
    ui.camera_img->setPixmap(qnode.getPixmap());
}

MainWindow::~MainWindow() {}

void MainWindow::on_button_start_calibration_clicked(bool check)
{
    std::cout << "Calibration Clicked\n";
    ui.button_start_calibration->setEnabled(false);
}



/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    ros::init(argc, argv, "bundle_calibration");
    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}

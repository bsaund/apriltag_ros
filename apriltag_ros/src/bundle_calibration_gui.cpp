#include "apriltag_ros/bundle_calibration_gui.hpp"

#include <QApplication>
#include <QtWidgets>


using namespace apriltag_ros;

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    // ReadSettings();
    // setWindowIcon(QIcon(":/images/icon.png"));
    // ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    // QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

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
}

MainWindow::~MainWindow() {}



/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

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

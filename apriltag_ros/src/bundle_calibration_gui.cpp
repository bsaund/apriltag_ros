#include "apriltag_ros/bundle_calibration_gui.hpp"



#include <QApplication>
#include <QtWidgets>
#include <QCheckBox>
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
    QObject::connect(&qnode, SIGNAL(newTagObserved(int)), this, SLOT(addTagToChecklist(int)));

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
    setupBundleCheckboxes();
}

void MainWindow::setupBundleCheckboxes()
{
    std::vector<TagBundleDescription> tag_bundle_descriptions = qnode.getTagBundleDescriptions();

    int checkbox_id = 0;
    for(const TagBundleDescription& bundle: tag_bundle_descriptions)
    {
        checkbox_id++;
        QString display_text = QString(bundle.name().c_str());
        QCheckBox* checkbox = new QCheckBox(display_text, ui.tag_selection);
        checkbox->move(0, checkbox_offset);
        checkbox->show();
        checkbox->setEnabled(false);
        checkbox->setToolTip("Cannot select this bundle until all tags have been observed");
        checkboxes.addButton(checkbox, checkbox_id);



        checkbox_offset += 5;
        for(int id: bundle.bundleIds())
        {
            if(specified_tags.count(id))
            {
                QMessageBox box;
                QString box_text = QString(("Tag " + std::to_string(id) + " appears in multiple bundles! Cannot calibrate").c_str());
                box.setText(box_text);
                box.exec();
            }
            
            checkbox_offset += 20;
            QString tag_text = QString(("Tag " + std::to_string(id)).c_str());
            specified_tags.emplace(std::piecewise_construct,
                                   std::forward_as_tuple(id),
                                   std::forward_as_tuple(tag_text, ui.tag_selection));
            specified_tags[id].move(30, checkbox_offset);
            specified_tags[id].show();
            specified_tags[id].setEnabled(false);
            specified_tags[id].setToolTip("This tag has not been seen yet");
                                    
            std::cout << "tag " << id << "\n";
        }
        checkbox_offset += 25;
    }

    QLabel* no_bundle_label = new QLabel("No Bundle", ui.tag_selection);
    no_bundle_label->move(0, checkbox_offset);
    checkbox_offset += 25;
    no_bundle_label->show();
}

void MainWindow::addTagToChecklist(int id)
{
    std::cout << "Detected new tag: " << id << "\n";

    if(specified_tags.count(id))
    {
        specified_tags[id].setEnabled(true);
        specified_tags[id].setToolTip("");

        std::vector<TagBundleDescription> tag_bundle_descriptions = qnode.getTagBundleDescriptions();

        int checkbox_id = 0;
        for(const TagBundleDescription& bundle: tag_bundle_descriptions)
        {
            checkbox_id++;
            bool all_tags_in_bundle_seen = true;
            for(int id: bundle.bundleIds())
            {
                if(!specified_tags[id].isEnabled())
                {
                    all_tags_in_bundle_seen = false;
                    break;
                }
            }
            if(all_tags_in_bundle_seen)
            {
                QAbstractButton* checkbox = checkboxes.button(checkbox_id);
                checkbox->setEnabled(true);
                checkbox->setToolTip("Check box to select this bundle for calibration");
            }
        }
        
        return;
    }

    
    QString display_text = QString(("Tag " + std::to_string(id)).c_str());
    // QCheckBox* checkbox = new QCheckBox(display_text, ui.tag_selection);
    // checkbox->move(0, checkbox_offset);
    // checkbox_offset += 25;
    // checkbox->setChecked(true);
    // checkbox->show();
    // checkbox->setEnabled(false);
    // checkboxes.addButton(checkbox, id);
    QLabel* tag_label = new QLabel(display_text, ui.tag_selection);
    tag_label->move(30, checkbox_offset);
    checkbox_offset += 20;
    tag_label->show();
}

void MainWindow::updateDisplayedImage()
{
    ui.camera_img->setPixmap(qnode.getPixmap());

    std::set<int> tags_detected = qnode.visible_tags;
    QString tags_detected_text("Visible Detected: [");
    for(int t: tags_detected)
    {
        tags_detected_text.append((std::to_string(t) + ", ").c_str());
    }
    tags_detected_text.append("]");
    ui.tags_detected->setText(tags_detected_text);

    QString calibration_text(("Num Calibration Points: " + std::to_string(qnode.calibration_data.size())).c_str());
    ui.num_calibration_points->setText(calibration_text);
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

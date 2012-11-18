/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <iostream>
#include "../include/kobuki_factory_test/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_factory_test {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    , msg_box(this)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** QNode interaction
    **********************/
//    ui.view_logging->setModel(qnode.loggingModel());
//    QObject::connect(&qnode, SIGNAL(showMessage(const QString&, const QString&)),
  //                     this,   SLOT(showPopupMsg(const QString&, const QString&)));
//    QObject::connect(&qnode, SIGNAL(hideMessage()), this, SLOT(hidePopupMsg()));
    QObject::connect(&qnode, SIGNAL(requestMW(QNodeRequest*)),
                         this, SLOT(qNodeRequest(QNodeRequest*)));
    QObject::connect(&qnode, SIGNAL(evalUpdated(Robot*)),
                         this, SLOT(updateEvalState(Robot*)));
    /*********************
    ** Logging
    **********************/
  //  ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(addLogLine(const QString&)),
                       this,  SLOT(viewLogLine(const QString&)));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    }/* else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }*/
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  //ui.view_logging->clear();

  QStringList sl = qnode.loggingModel()->stringList();
  for (int i = 0; i < qnode.loggingModel()->rowCount(); i++) {
    ui.view_logging->addItem("ggg");
    ui.view_logging->item(i)->setForeground(Qt::red);
  }

  ui.view_logging->scrollToBottom();
}

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::viewLogLine(const QString& str) {
  QListWidgetItem* item = new QListWidgetItem(str);
  if ((str.contains("[ERROR]")) || (str.contains("[FATAL]")))
    item->setForeground(Qt::darkRed);
  else if (str.contains("[WARN]"))
    item->setForeground(Qt::darkYellow);
  else if (str.contains("[INFO]"))
    item->setForeground(Qt::black);
  else if (str.contains("[DEBUG]"))
    item->setForeground(Qt::darkGreen);

  ui.view_logging->addItem(item);
  ui.view_logging->scrollToBottom();
}

void MainWindow::qNodeRequest(QNodeRequest* request) {
  QCoreApplication::postEvent(this, request);
}

bool MainWindow::event(QEvent* e) {
  if (e->type() == QEvent::User) {
    QNodeRequest* request = (QNodeRequest*)e;

    if ((request->title.isEmpty() == false) || (request->text.isEmpty() == false)) {
      msg_box.setText(request->text);
      msg_box.setWindowTitle(request->title);
      msg_box.show();
    }
    else
      msg_box.hide();
  }
  else
    QMainWindow::event(e);

  return true;
}

QString& styleSheet(double n, double N) {
  QString bcrgb;
  return bcrgb.sprintf("background-color: rgb(%d, %d, %d);", int(((N - n)/N)*255), 0, int((n/N)*255));
}

QString getStyleSheet(float completed) {
  QString bcrgb;
  return bcrgb.sprintf("background-color: rgb(%d, %d, %d);", int((1 - completed)*255), 0, int(completed*255));
}

//#define EVAL_0  setStyleSheet("background-color: rgb(255, 0, 0);");
//#define EVAL_1  setStyleSheet("background-color: rgb(255, 110, 20);");
//#define EVAL_2  setStyleSheet("background-color: rgb(0, 0, 255);");

void MainWindow::updateEvalState(Robot* robot) {
  // Left column
  ui.ir_dock_widget->setStyleSheet(getStyleSheet((robot->device_cmp[Robot::IR_DOCK_L] + robot->device_cmp[Robot::IR_DOCK_C] + robot->device_cmp[Robot::IR_DOCK_R])/3.0));
  ui.imu_dev_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::IMU_DEV]));
  ui.buttons_widget->setStyleSheet(getStyleSheet((robot->device_cmp[Robot::BUTTON_0] + robot->device_cmp[Robot::BUTTON_1] + robot->device_cmp[Robot::BUTTON_2])/3.0));
  ui.bumper_l_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::BUMPER_L]));
  ui.bumper_c_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::BUMPER_C]));
  ui.bumper_r_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::BUMPER_R]));
  ui.w_drop_l_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::W_DROP_L]));
  ui.w_drop_r_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::W_DROP_R]));
  ui.cliff_l_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::CLIFF_L]));
  ui.cliff_c_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::CLIFF_C]));
  ui.cliff_r_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::CLIFF_R]));

  // Right column
  ui.pwr_jack_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::PWR_JACK]));
  ui.pwr_dock_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::PWR_DOCK]));
  ui.charging_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::CHARGING]));
  ui.ext_pwr_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::EXT_PWR]));
  ui.a_input_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::A_INPUT]));
  ui.d_input_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::D_INPUT]));
  ui.d_output_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::D_OUTPUT]));
  ui.leds_widget->setStyleSheet(getStyleSheet((robot->device_cmp[Robot::LED_1] + robot->device_cmp[Robot::LED_2])/2.0f));
  ui.sounds_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::SOUNDS]));
  ui.motor_l_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::MOTOR_L]));
  ui.motor_r_widget->setStyleSheet(getStyleSheet(robot->device_cmp[Robot::MOTOR_R]));

  QListWidgetItem* item = new QListWidgetItem();
  item->setBackground(Qt::darkRed);
//  ui.devStatesListWidget->addItem(item);
//  robot->
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "kobuki_factory_test");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "kobuki_factory_test");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace kobuki_factory_test


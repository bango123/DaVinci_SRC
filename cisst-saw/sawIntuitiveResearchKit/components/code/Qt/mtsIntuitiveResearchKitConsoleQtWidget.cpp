/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-17

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>

#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>
#include <QTextEdit>
#include <QScrollBar>
#include <QGroupBox>
#include <QTabWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTime>
#include <QLabel>
#include <QPixmap>
#include <QShortcut>
#include <QDoubleSpinBox>
#include <QCheckBox>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQtWidget);

mtsIntuitiveResearchKitConsoleQtWidget::mtsIntuitiveResearchKitConsoleQtWidget(const std::string & componentName):
    mtsComponent(componentName), m_isRecording(false)
{
    QObject *parent;
    QProcess_Recording = new QProcess(parent);

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Main");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("PowerOff", Console.PowerOff);
        interfaceRequired->AddFunction("Home", Console.Home);
        interfaceRequired->AddFunction("TeleopEnable", Console.TeleopEnable);
        interfaceRequired->AddFunction("SetScale", Console.SetScale);
        interfaceRequired->AddFunction("SetDelay", Console.SetDelay);
        interfaceRequired->AddFunction("SetRosOnly", Console.SetRosOnly);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::RosOnlyEventHandler,
                                                this, "RosOnly");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ScaleEventHandler,
                                                this, "Scale");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::DelayEventHandler,
                                                this, "Delay");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ErrorEventHandler,
                                                    this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::WarningEventHandler,
                                                    this, "Warning");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::StatusEventHandler,
                                                    this, "Status");
    }
    setupUi();
}

void mtsIntuitiveResearchKitConsoleQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsoleQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsoleQtWidget::Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::HasTeleOp(const bool & hasTeleOp)
{
    QPBTeleopStart->setEnabled(hasTeleOp);
    QPBTeleopStop->setEnabled(hasTeleOp);
    QSBScale->setEnabled(hasTeleOp);
    QSBDelay->setEnabled(hasTeleOp);
    QCBRosOnly->setEnabled(hasTeleOp);
}

void mtsIntuitiveResearchKitConsoleQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsIntuitiveResearchKitConsoleQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        this->hide();
        // send clean power off message and wait a bit
        Console.PowerOff();
        osaSleep(2.0 * cmn_s);
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotPowerOff(void)
{
    Console.PowerOff();
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotHome(void)
{
    Console.Home();
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTeleopStart(void)
{
    Console.TeleopEnable(true);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTeleopStop(void)
{
    Console.TeleopEnable(false);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotToggleRecord(void){
    m_isRecording = !m_isRecording;

    if(m_isRecording){
        QString command = "roslaunch dvrk_robot dvrk_camera_console_record.launch filepath:=" + QTFileName->toPlainText();
        QProcess_Recording->start(command);

        QPBRecord->setText("Stop Recording");
    }
    else{
        QProcess_Recording->terminate();
        QPBRecord->setText("Start Recording");
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotSetScale(double scale)
{
    Console.SetScale(scale);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotSetDelay(double delay)
{
    Console.SetDelay(delay);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotRosOnly(bool rosOnly)
{
    Console.SetRosOnly(rosOnly);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsIntuitiveResearchKitConsoleQtWidget::setupUi(void)
{
    QHBoxLayout * mainLayout = new QHBoxLayout;

    QWidget * buttonsWidget = new QWidget();
    QVBoxLayout * boxLayout = new QVBoxLayout();
    boxLayout->setContentsMargins(0, 0, 0, 0);
    buttonsWidget->setLayout(boxLayout);

    QGroupBox * powerBox = new QGroupBox("Power");
    boxLayout->addWidget(powerBox);
    QVBoxLayout * powerLayout = new QVBoxLayout();
    powerBox->setLayout(powerLayout);
    QPBPowerOff = new QPushButton("Off");
    QPBPowerOff->setToolTip("ctrl + O");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_O), this, SLOT(SlotPowerOff()));
    powerLayout->addWidget(QPBPowerOff);
    QPBHome = new QPushButton("Home");
    QPBHome->setToolTip("ctrl + H");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_H), this, SLOT(SlotHome()));
    powerLayout->addWidget(QPBHome);

    QGroupBox * teleopBox = new QGroupBox("Teleop");
    boxLayout->addWidget(teleopBox);
    QVBoxLayout * teleopLayout = new QVBoxLayout();
    teleopBox->setLayout(teleopLayout);

    QPBTeleopStart = new QPushButton("Start");
    QPBTeleopStart->setToolTip("ctrl + T");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_T), this, SLOT(SlotTeleopStart()));
    teleopLayout->addWidget(QPBTeleopStart);

    QPBTeleopStop = new QPushButton("Stop");
    QPBTeleopStop->setToolTip("ctrl + S");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_S), this, SLOT(SlotTeleopStop()));
    teleopLayout->addWidget(QPBTeleopStop);

    QSBScale = new QDoubleSpinBox();
    QSBScale->setDecimals(4);
    QSBScale->setRange(0.05, 1.0);
    QSBScale->setSingleStep(0.05);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(0.2);
    teleopLayout->addWidget(QSBScale);

    QSBDelay = new QDoubleSpinBox();
    QSBDelay->setDecimals(0);
    QSBDelay->setRange(0, 500);
    QSBDelay->setSingleStep(10);
    QSBDelay->setPrefix("Delay (ms) ");
    QSBDelay->setValue(0);
    teleopLayout->addWidget(QSBDelay);

    QCBRosOnly = new QCheckBox("Ros Only");
    QCBRosOnly->setChecked(false);
    teleopLayout->addWidget(QCBRosOnly);

    QTFileName = new QPlainTextEdit("Filename");
    teleopLayout->addWidget(QTFileName);

    QPBRecord = new QPushButton("Start Recording");
    QPBRecord->setToolTip("ctrl + R");
    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_R), this, SLOT(SlotToggleRecord()));
    teleopLayout->addWidget(QPBRecord);

    boxLayout->addStretch(100);
    buttonsWidget->setFixedWidth(buttonsWidget->sizeHint().width());
    mainLayout->addWidget(buttonsWidget);

    QLabel * labelLogo = new QLabel("");
    labelLogo->setPixmap(QPixmap(":/dVRK.svg"));
    boxLayout->addWidget(labelLogo);

    QSplitter * tabWidgetAndMessages = new QSplitter();
    tabWidgetAndMessages->setOrientation(Qt::Vertical);

    QTWidgets = new QTabWidget();
    tabWidgetAndMessages->addWidget(QTWidgets);

    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    QTEMessages->resize(QTEMessages->width(), 600);
    tabWidgetAndMessages->addWidget(QTEMessages);

    mainLayout->addWidget(tabWidgetAndMessages);
    setLayout(mainLayout);

    std::string title = "dVRK ";
    title.append(sawIntuitiveResearchKit_VERSION);
    title.append(" / cisst ");
    title.append(CISST_VERSION);
    setWindowTitle(title.c_str());
    resize(sizeHint());

    // buttons
    connect(QPBPowerOff, SIGNAL(clicked()),
            this, SLOT(SlotPowerOff()));
    connect(QPBHome, SIGNAL(clicked()),
            this, SLOT(SlotHome()));
    connect(QPBTeleopStart, SIGNAL(clicked()),
            this, SLOT(SlotTeleopStart()));
    connect(QPBTeleopStop, SIGNAL(clicked()),
            this, SLOT(SlotTeleopStop()));
    connect(QPBRecord, SIGNAL(clicked()),
            this, SLOT(SlotToggleRecord()));

    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetScale(double)));
    connect(this, SIGNAL(SignalScale(double)),
            this, SLOT(SlotScaleEventHandler(double)));

    connect(QSBDelay, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetDelay(double)));
    connect(this, SIGNAL(SignalDelay(double)),
            this, SLOT(SlotDelayEventHandler(double)));
    
    connect(QCBRosOnly, SIGNAL(clicked(bool)),
            this, SLOT(SlotRosOnly(bool)));
    connect(this, SIGNAL(SignalRosOnly(bool)),
            this, SLOT(SlotRosOnlyEventHandler(bool)));


    // messages
    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));

    new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_Q), this, SLOT(close()));

}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsIntuitiveResearchKitConsoleQtWidget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotDelayEventHandler(double delay){
    QSBDelay->setValue(delay);
}

void mtsIntuitiveResearchKitConsoleQtWidget::DelayEventHandler(const double & delay)
{
    emit SignalDelay(delay);
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotRosOnlyEventHandler(bool rosOnly){
    QCBRosOnly->setChecked(rosOnly);
}

void mtsIntuitiveResearchKitConsoleQtWidget::RosOnlyEventHandler(const bool & rosOnly){
    emit SignalRosOnly(rosOnly);
}


void mtsIntuitiveResearchKitConsoleQtWidget::ErrorEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("red"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
}

void mtsIntuitiveResearchKitConsoleQtWidget::WarningEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("darkRed"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Warning: ") + QString(message.c_str()));
}

void mtsIntuitiveResearchKitConsoleQtWidget::StatusEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("black"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
}

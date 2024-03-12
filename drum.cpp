#include "drum.h"
#include "ui_drum.h"

#include <QFileDialog>
#include <QtMath>
#include <QMetaType>

#include <QDebug>


Drum::Drum(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Drum)
{
    ui->setupUi(this);



    initChart();
    initUi();

    init();

}

Drum::~Drum()
{
    m_pTimerUI->stop();

    m_StateThread.join();
    m_SendThread.join();
    m_ReceiveThread.join();


    delete ui;
}

void Drum::initUi()
{

    ui->label_can1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_can2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_can3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_l4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_r4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_status_waist->setStyleSheet("background-color: dimgrey; border-radius: 12px;");

    ui->label_home_done_l1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_l2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_l3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_l4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_r4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    ui->label_home_done_waist->setStyleSheet("background-color: dimgrey; border-radius: 12px;");

    ui->comboBox_state_demand->setCurrentIndex(-1);
    ui->comboBox_view_pos->setCurrentIndex(-1);
    ui->comboBox_view_vel->setCurrentIndex(-1);
    ui->comboBox_view_tor->setCurrentIndex(-1);


    ui->comboBox_home_joint->setCurrentIndex(-1);
    ui->tabWidget->setCurrentIndex(0);
}

void Drum::init()
{

    m_pTimerUI = new QTimer();
    m_pTimerUI->setInterval(300);

    m_pCanManager = new CanManager(m_Motors);
    m_pPathManager = new PathManager(m_SystemState,
                                     *m_pCanManager,
                                     m_Motors);

    m_pTestManager = new TestManager(m_SystemState,
                                     *m_pCanManager,
                                     m_Motors);

    m_pHomeManager = new HomeManager(m_SystemState,
                                     *m_pCanManager,
                                     m_Motors);

    m_pDrumRobot = new DrumRobot(m_SystemState,
                                 *m_pCanManager,
                                 *m_pPathManager,
                                 *m_pHomeManager,
                                 *m_pTestManager,
                                 m_Motors);


    initConnections();

    m_StateThread = std::thread(&DrumRobot::stateMachine, m_pDrumRobot);
    m_SendThread = std::thread(&DrumRobot::sendLoopForThread, m_pDrumRobot);
    m_ReceiveThread = std::thread(&DrumRobot::recvLoopForThread, m_pDrumRobot);



    m_pTimerUI->start();



}

void Drum::initConnections()
{
    //QObject::connect(&m_pDrumRobot->m_Signals, &Signals::stateChanged, this, &Drum::setState);
    //QObject::connect(&m_pDrumRobot->m_Signals, &Signals::motorInfosUpdated, this, &Drum::setMotorInfos);
    //QObject::connect(&m_pDrumRobot->m_Signals, &Signals::canStateChanged, this, &Drum::setCanState);

    QObject::connect(m_pTimerUI, &QTimer::timeout, this, &Drum::updateUI);
    QObject::connect(ui->comboBox_state_demand, &QComboBox::currentTextChanged, this, &Drum::on_comboBox_state_demand_currentTextChanged);
    QObject::connect(ui->pushButton_music, &QPushButton::clicked, this, &Drum::on_pushButton_music_clicked);
    //QObject::connect(ui->pushButton_start, SIGNAL(clicked()), this, SLOT(on_pushButton_start_clicked()));
    QObject::connect(ui->pushButton_start, &QPushButton::clicked, this, &Drum::on_pushButton_start_clicked);
    QObject::connect(ui->comboBox_view_pos, &QComboBox::currentTextChanged, this, &Drum::on_comboBox_view_pos_currentTextChanged);
    QObject::connect(ui->comboBox_view_vel, &QComboBox::currentTextChanged, this, &Drum::on_comboBox_view_vel_currentTextChanged);
    QObject::connect(ui->comboBox_view_tor, &QComboBox::currentTextChanged, this, &Drum::on_comboBox_view_tor_currentTextChanged);

    QObject::connect(ui->pushButton_all_home, &QPushButton::clicked, this, &Drum::on_pushButton_all_home_clicked);
    QObject::connect(ui->pushButton_home, &QPushButton::clicked, this, &Drum::on_pushButton_home_clicked);

    //QObject::connect(&m_pDrumRobot->m_Signals, &Signals::homingDone, this, &Drum::setHomeDone);

}

void Drum::initChart()
{


    // 위치(Position) 차트 초기화
    m_pChartPos = new QChart();
    m_pChartVel = new QChart();
    m_pChartTor = new QChart();

    // 왼쪽 모터의 위치에 대한 라인 시리즈 생성 및 차트에 추가
    m_pSeriesPos_L = new QLineSeries();
    m_pSeriesPos_L->setName("Left Motor Position"); // 시리즈 이름 설정 (옵션)
    m_pChartPos->addSeries(m_pSeriesPos_L); // 왼쪽 모터

    m_pSeriesVel_L = new QLineSeries();
    m_pSeriesVel_L->setName("Left Motor Velocity"); // 시리즈 이름 설정 (옵션)
    m_pChartVel->addSeries(m_pSeriesVel_L); // 왼쪽 모터

    m_pSeriesTor_L = new QLineSeries();
    m_pSeriesTor_L->setName("Left Motor Torque"); // 시리즈 이름 설정 (옵션)
    m_pChartTor->addSeries(m_pSeriesTor_L); // 왼쪽 모터

    // 오른쪽 모터에 대한 라인 시리즈 생성 및 차트에 추가
    m_pSeriesPos_R = new QLineSeries();
    m_pSeriesPos_R->setName("Right Motor Position"); // 시리즈 이름 설정 (옵션)
    m_pChartPos->addSeries(m_pSeriesPos_R); // 오른쪽 모터 시리즈를 차트에 추가

    m_pSeriesVel_R = new QLineSeries();
    m_pSeriesVel_R->setName("Right Motor Velocity"); // 시리즈 이름 설정 (옵션)
    m_pChartVel->addSeries(m_pSeriesVel_R); // 오른쪽 모터 시리즈를 차트에 추가

    m_pSeriesTor_R = new QLineSeries();
    m_pSeriesTor_R->setName("Right Motor Torque"); // 시리즈 이름 설정 (옵션)
    m_pChartTor->addSeries(m_pSeriesTor_R); // 오른쪽 모터 시리즈를 차트에 추가



    // 허리 모터에 대한 라인 시리즈 생성 및 차트에 추가
    m_pSeriesPos_waist = new QLineSeries();
    m_pSeriesPos_waist->setName("Waist Motor Position"); // 시리즈 이름 설정 (옵션)
    m_pChartPos->addSeries(m_pSeriesPos_waist); // 허리 모터 시리즈를 차트에 추가

    m_pSeriesVel_waist = new QLineSeries();
    m_pSeriesVel_waist->setName("Waist Motor Velocity"); // 시리즈 이름 설정 (옵션)
    m_pChartVel->addSeries(m_pSeriesVel_waist); // 허리 모터 시리즈를 차트에 추가

    m_pSeriesTor_waist = new QLineSeries();
    m_pSeriesTor_waist->setName("Waist Motor Torque"); // 시리즈 이름 설정 (옵션)
    m_pChartTor->addSeries(m_pSeriesTor_waist); // 허리 모터 시리즈를 차트에 추가

    // X축 및 Y축 생성 및 범위 설정
    m_pAxisPosX = new QValueAxis();
    m_pAxisPosY = new QValueAxis();
    m_pAxisPosX->setRange(0, 100); // X축 범위 설정
    m_pAxisPosY->setRange(-100, 100); // Y축 범위 설정

    m_pAxisVelX = new QValueAxis();
    m_pAxisVelY = new QValueAxis();
    m_pAxisVelX->setRange(0, 100); // X축 범위 설정
    m_pAxisVelY->setRange(-100, 100); // Y축 범위 설정

    m_pAxisTorX = new QValueAxis();
    m_pAxisTorY = new QValueAxis();
    m_pAxisTorX->setRange(0, 100); // X축 범위 설정
    m_pAxisTorY->setRange(-100, 100); // Y축 범위 설정

    // 생성된 축을 차트에 적용
    m_pChartPos->setAxisX(m_pAxisPosX, m_pSeriesPos_L); // 왼쪽 모터 시리즈에 X축 적용
    m_pChartPos->setAxisY(m_pAxisPosY, m_pSeriesPos_L); // 왼쪽 모터 시리즈에 Y축 적용
    m_pChartPos->setAxisX(m_pAxisPosX, m_pSeriesPos_R); // 오른쪽 모터 시리즈에도 동일한 X축 적용
    m_pChartPos->setAxisY(m_pAxisPosY, m_pSeriesPos_R); // 오른쪽 모터 시리즈에도 동일한 Y축 적용
    m_pChartPos->setAxisX(m_pAxisPosX, m_pSeriesPos_waist); // 오른쪽 모터 시리즈에도 동일한 X축 적용
    m_pChartPos->setAxisY(m_pAxisPosY, m_pSeriesPos_waist); // 오른쪽 모터 시리즈에도 동일한 Y축 적용

    m_pChartVel->setAxisX(m_pAxisVelX, m_pSeriesVel_L); // 왼쪽 모터 시리즈에 X축 적용
    m_pChartVel->setAxisY(m_pAxisVelY, m_pSeriesVel_L); // 왼쪽 모터 시리즈에 Y축 적용
    m_pChartVel->setAxisX(m_pAxisVelX, m_pSeriesVel_R); // 오른쪽 모터 시리즈에도 동일한 X축 적용
    m_pChartVel->setAxisY(m_pAxisVelY, m_pSeriesVel_R); // 오른쪽 모터 시리즈에도 동일한 Y축 적용
    m_pChartVel->setAxisX(m_pAxisVelX, m_pSeriesVel_waist); // 오른쪽 모터 시리즈에도 동일한 X축 적용
    m_pChartVel->setAxisY(m_pAxisVelY, m_pSeriesVel_waist); // 오른쪽 모터 시리즈에도 동일한 Y축 적용

    m_pChartTor->setAxisX(m_pAxisTorX, m_pSeriesTor_L); // 왼쪽 모터 시리즈에 X축 적용
    m_pChartTor->setAxisY(m_pAxisTorY, m_pSeriesTor_L); // 왼쪽 모터 시리즈에 Y축 적용
    m_pChartTor->setAxisX(m_pAxisTorX, m_pSeriesTor_R); // 오른쪽 모터 시리즈에도 동일한 X축 적용
    m_pChartTor->setAxisY(m_pAxisTorY, m_pSeriesTor_R); // 오른쪽 모터 시리즈에도 동일한 Y축 적용
    m_pChartTor->setAxisX(m_pAxisTorX, m_pSeriesTor_waist); // 오른쪽 모터 시리즈에도 동일한 X축 적용
    m_pChartTor->setAxisY(m_pAxisTorY, m_pSeriesTor_waist); // 오른쪽 모터 시리즈에도 동일한 Y축 적용

    // 차트의 애니메이션 옵션 설정
    m_pChartPos->setAnimationOptions(QChart::AllAnimations);
    m_pChartVel->setAnimationOptions(QChart::AllAnimations);
    m_pChartTor->setAnimationOptions(QChart::AllAnimations);

    // 차트를 UI의 QGraphicsView 위젯에 설정
    ui->graphicsView_pos->setChart(m_pChartPos);
    ui->graphicsView_pos->setRenderHints(QPainter::Antialiasing); // 차트의 안티앨리어싱 활성화

    ui->graphicsView_vel->setChart(m_pChartVel);
    ui->graphicsView_vel->setRenderHints(QPainter::Antialiasing); // 차트의 안티앨리어싱 활성화

    ui->graphicsView_tor->setChart(m_pChartTor);
    ui->graphicsView_tor->setRenderHints(QPainter::Antialiasing); // 차트의 안티앨리어싱 활성화

}

void Drum::setMainState()
{
    switch (m_SystemState.main) {

    case Main::SystemInit:
        ui->lineEdit_cur_state->setText("SystemInit");
        break;
    case Main::Ideal:
        ui->lineEdit_cur_state->setText("IDLE");
        break;
    case Main::Homing:
        ui->lineEdit_cur_state->setText("Homing");
        break;

    case Main::Tune:
        ui->lineEdit_cur_state->setText("Tune");
        break;

    case Main::Perform:
        ui->lineEdit_cur_state->setText("Perform");
        break;

    case Main::Check:
        ui->lineEdit_cur_state->setText("Check");
        break;

    case Main::Shutdown:
        ui->lineEdit_cur_state->setText("Shutdown");
        break;

    case Main::Back:
        ui->lineEdit_cur_state->setText("Back");
        break;

    case Main::Ready:
        ui->lineEdit_cur_state->setText("Ready");
        break;

    }

}

void Drum::setMotorInfos()
{


    if (m_Motors.find("L_arm1") != m_Motors.end()) {
        ui->lineEdit_pos_l1->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm1"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l1->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm1"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l1->setText(QString::number(m_Motors["L_arm1"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l1->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm1"]->desPos), 'f', 2));
    }

    if (m_Motors.find("L_arm2") != m_Motors.end()) {
        ui->lineEdit_pos_l2->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm2"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l2->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm2"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l2->setText(QString::number(m_Motors["L_arm2"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l2->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm2"]->desPos), 'f', 2));
    }

    if (m_Motors.find("L_arm3") != m_Motors.end()) {
        ui->lineEdit_pos_l3->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm3"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l3->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm3"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l3->setText(QString::number(m_Motors["L_arm3"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l3->setText(QString::number(qRadiansToDegrees(m_Motors["L_arm3"]->desPos), 'f', 2));
    }

    if (m_Motors.find("L_wrist") != m_Motors.end()) {
        ui->lineEdit_pos_l4->setText(QString::number(qRadiansToDegrees(m_Motors["L_wrist"]->currentPos), 'f', 2));
        ui->lineEdit_vel_l4->setText(QString::number(qRadiansToDegrees(m_Motors["L_wrist"]->currentVel), 'f', 2));
        ui->lineEdit_tor_l4->setText(QString::number(m_Motors["L_wrist"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_l4->setText(QString::number(qRadiansToDegrees(m_Motors["L_wrist"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_arm1") != m_Motors.end()) {
        ui->lineEdit_pos_r1->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm1"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r1->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm1"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r1->setText(QString::number(m_Motors["R_arm1"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r1->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm1"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_arm2") != m_Motors.end()) {
        ui->lineEdit_pos_r2->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm2"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r2->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm2"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r2->setText(QString::number(m_Motors["R_arm2"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r2->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm2"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_arm3") != m_Motors.end()) {
        ui->lineEdit_pos_r3->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm3"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r3->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm3"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r3->setText(QString::number(m_Motors["R_arm3"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r3->setText(QString::number(qRadiansToDegrees(m_Motors["R_arm3"]->desPos), 'f', 2));
    }

    if (m_Motors.find("R_wrist") != m_Motors.end()) {
        ui->lineEdit_pos_r4->setText(QString::number(qRadiansToDegrees(m_Motors["R_wrist"]->currentPos), 'f', 2));
        ui->lineEdit_vel_r4->setText(QString::number(qRadiansToDegrees(m_Motors["R_wrist"]->currentVel), 'f', 2));
        ui->lineEdit_tor_r4->setText(QString::number(m_Motors["R_wrist"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_r4->setText(QString::number(qRadiansToDegrees(m_Motors["R_wrist"]->desPos), 'f', 2));
    }

    if (m_Motors.find("waist") != m_Motors.end()) {
        ui->lineEdit_pos_waist->setText(QString::number(qRadiansToDegrees(m_Motors["waist"]->currentPos), 'f', 2));
        ui->lineEdit_vel_waist->setText(QString::number(qRadiansToDegrees(m_Motors["waist"]->currentVel), 'f', 2));
        ui->lineEdit_tor_waist->setText(QString::number(m_Motors["waist"]->currentTor, 'f', 2));
        ui->lineEdit_pos_dem_waist->setText(QString::number(qRadiansToDegrees(m_Motors["waist"]->desPos), 'f', 2));
    }



}

void Drum::setMotorStatus()
{
    if (m_Motors.find("L_arm1") != m_Motors.end()) {
        if (m_Motors["L_arm1"]->isConected) {
            ui->label_status_l1->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_l1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("L_arm2") != m_Motors.end()) {
        if (m_Motors["L_arm2"]->isConected) {
            ui->label_status_l2->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_l2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("L_arm3") != m_Motors.end()) {
        if (m_Motors["L_arm3"]->isConected) {
            ui->label_status_l3->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_l3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("R_arm1") != m_Motors.end()) {
        if (m_Motors["R_arm1"]->isConected) {
            ui->label_status_r1->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_r1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("R_arm2") != m_Motors.end()) {
        if (m_Motors["R_arm2"]->isConected) {
            ui->label_status_r2->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_r2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("R_arm3") != m_Motors.end()) {
        if (m_Motors["R_arm3"]->isConected) {
            ui->label_status_r3->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_r3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("waist") != m_Motors.end()) {
        if (m_Motors["waist"]->isConected) {
            ui->label_status_waist->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_waist->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("L_wrist") != m_Motors.end()) {
        if (m_Motors["L_wrist"]->isConected) {
            ui->label_status_l4->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_l4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    if (m_Motors.find("R_wrist") != m_Motors.end()) {
        if (m_Motors["R_wrist"]->isConected) {
            ui->label_status_r4->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else {
            ui->label_status_r4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
}

void Drum::setCanState()
{

    int size = m_pCanManager->ifnames.size();

    if (m_pCanManager->isConnected[m_pCanManager->ifnames[0]]) {
        ui->label_can1->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
    }
    else {
        ui->label_can1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    }

    if (m_pCanManager->isConnected[m_pCanManager->ifnames[1]]) {
        ui->label_can2->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
    }
    else {
        ui->label_can2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    }

    if (m_pCanManager->isConnected[m_pCanManager->ifnames[2]]) {
        ui->label_can3->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
    }
    else {
        ui->label_can3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
    }
}

void Drum::setHomeDone()
{
    if (m_Motors.find("L_arm1") != m_Motors.end()) {
        if (m_Motors["L_arm1"]->isHomed) {
            ui->label_home_done_l1->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_l1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("L_arm2") != m_Motors.end()) {
        if (m_Motors["L_arm2"]->isHomed) {
            ui->label_home_done_l2->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_l2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("L_arm3") != m_Motors.end()) {
        if (m_Motors["L_arm3"]->isHomed) {
            ui->label_home_done_l3->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_l3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("R_arm1") != m_Motors.end()) {
        if (m_Motors["R_arm1"]->isHomed) {
            ui->label_home_done_r1->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_r1->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("R_arm2") != m_Motors.end()) {
        if (m_Motors["R_arm2"]->isHomed) {
            ui->label_home_done_r2->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_r2->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("R_arm3") != m_Motors.end()) {
        if (m_Motors["R_arm3"]->isHomed) {
            ui->label_home_done_r3->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_r3->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("L_wrist") != m_Motors.end()) {
        if (m_Motors["L_wrist"]->isHomed) {
            ui->label_home_done_l4->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_l4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("R_wrist") != m_Motors.end()) {
        if (m_Motors["R_wrist"]->isHomed) {
            ui->label_home_done_r4->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_r4->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
    else if (m_Motors.find("waist") != m_Motors.end()) {
        if (m_Motors["waist"]->isHomed) {
            ui->label_home_done_waist->setStyleSheet("background-color: lawngreen; border-radius: 12px;");
        }
        else{
            ui->label_home_done_waist->setStyleSheet("background-color: dimgrey; border-radius: 12px;");
        }
    }
}

void Drum::on_comboBox_state_demand_currentTextChanged(const QString &arg1)
{
    if (arg1 == "") {
        return;
    }

    if (arg1 == "HOME") {
        m_pDrumRobot->m_Input = "h";
    }
    else if (arg1 == "TUNE") {
        m_pDrumRobot->m_Input = "t";
    }
    else if (arg1 == "READY") {
        m_pDrumRobot->m_Input = "r";
    }
    else if (arg1 == "CHECK") {
        m_pDrumRobot->m_Input = "c";
    }
    else if (arg1 == "SHUTDOWN") {
        m_pDrumRobot->m_Input = "s";
    }
    else if (arg1 == "PERFORM") {
        m_pDrumRobot->m_Input = "p";
    }
    else if (arg1 == "BACK") {
        m_pDrumRobot->m_Input = "b";
    }

}


void Drum::on_pushButton_music_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "open File", "./", "File (*.*);;Text files(*.txt)");
}


void Drum::on_pushButton_start_clicked()
{
    int test = 1;
}


void Drum::setChartValue(){
    // 선택된 조인트의 왼쪽과 오른쪽 식별자 설정


    if (ui->tabWidget->tabText(ui->tabWidget->currentIndex()) == "Position")
    {
        std::string joint = ui->comboBox_view_pos->currentText().toStdString();
        std::string joint_L = "L_" + joint;
        std::string joint_R = "R_" + joint;

        if(joint == "waist") {
            if (m_Motors.find(joint) != m_Motors.end()) {
                qreal x_waist_pos = m_pSeriesPos_waist->count();
                qreal pos_waist = qRadiansToDegrees(m_Motors[joint]->currentPos);
                m_pSeriesPos_waist->append(x_waist_pos, pos_waist);

                if (x_waist_pos >100) m_pSeriesPos_waist->clear();
            }

        }else{
            // 왼쪽 모터의 위치 데이터 추가
            if (m_Motors.find(joint_L) != m_Motors.end()) {
                qreal x_L_pos = m_pSeriesPos_L->count(); // 왼쪽 모터 시리즈의 현재 데이터 개수를 X축 값으로 사용
                qreal pos_L = qRadiansToDegrees(m_Motors[joint_L]->currentPos); // 왼쪽 모터의 현재 위치 값을 변환
                m_pSeriesPos_L->append(x_L_pos, pos_L); // 왼쪽 모터 시리즈에 데이터 포인트 추가

                if (x_L_pos >100)  m_pSeriesPos_L->clear();

            }

            // 오른쪽 모터의 위치 데이터 추가
            if (m_Motors.find(joint_R) != m_Motors.end()) {
                qreal x_R_pos = m_pSeriesPos_R->count(); // 오른쪽 모터 시리즈의 현재 데이터 개수를 X축 값으로 사용
                qreal pos_R = qRadiansToDegrees(m_Motors[joint_R]->currentPos); // 오른쪽 모터의 현재 위치 값을 변환
                m_pSeriesPos_R->append(x_R_pos, pos_R); // 오른쪽 모터 시리즈에 데이터 포인트 추가

                if (x_R_pos >100) m_pSeriesPos_R->clear();

            }
        }

    }
    else if(ui->tabWidget->tabText(ui->tabWidget->currentIndex()) == "Velocity"){

        std::string joint = ui->comboBox_view_vel->currentText().toStdString();
        std::string joint_L = "L_" + joint;
        std::string joint_R = "R_" + joint;

        if(joint == "waist") {
            if (m_Motors.find(joint) != m_Motors.end()) {

                qreal x_waist_vel = m_pSeriesVel_waist->count();
                qreal vel_waist = qRadiansToDegrees(m_Motors[joint]->currentVel);
                m_pSeriesVel_waist->append(x_waist_vel, vel_waist);

                if (x_waist_vel >100) m_pSeriesVel_waist->clear();

            }

        }else{
            // 왼쪽 모터의 위치 데이터 추가
            if (m_Motors.find(joint_L) != m_Motors.end()) {

                qreal x_L_vel = m_pSeriesVel_waist->count();
                qreal vel_L = qRadiansToDegrees(m_Motors[joint]->currentVel);
                m_pSeriesVel_L->append(x_L_vel, vel_L);

                if (x_L_vel >100) m_pSeriesVel_L->clear();

            }

            // 오른쪽 모터의 위치 데이터 추가
            if (m_Motors.find(joint_R) != m_Motors.end()) {

                qreal x_R_vel = m_pSeriesVel_waist->count();
                qreal vel_R = qRadiansToDegrees(m_Motors[joint]->currentVel);
                m_pSeriesVel_R->append(x_R_vel, vel_R);

                if (x_R_vel >100) m_pSeriesVel_R->clear();

            }
        }
    }
    else if(ui->tabWidget->tabText(ui->tabWidget->currentIndex()) == "Torque"){

        std::string joint = ui->comboBox_view_tor->currentText().toStdString();
        std::string joint_L = "L_" + joint;
        std::string joint_R = "R_" + joint;

        if(joint == "waist") {
            if (m_Motors.find(joint) != m_Motors.end()) {

                qreal x_waist_tor = m_pSeriesTor_waist->count();
                qreal tor_waist = qRadiansToDegrees(m_Motors[joint]->currentTor);
                m_pSeriesTor_waist->append(x_waist_tor, tor_waist);

                if (x_waist_tor >100) {

                    m_pSeriesTor_waist->clear();
                }
            }

        }else{
            // 왼쪽 모터의 위치 데이터 추가
            if (m_Motors.find(joint_L) != m_Motors.end()) {

                qreal x_L_tor = m_pSeriesTor_waist->count();
                qreal tor_L = qRadiansToDegrees(m_Motors[joint]->currentTor);
                m_pSeriesTor_L->append(x_L_tor, tor_L);

                if (x_L_tor >100) {

                    m_pSeriesTor_L->clear();
                }

            }
            // 오른쪽 모터의 위치 데이터 추가
            if (m_Motors.find(joint_R) != m_Motors.end()) {

                qreal x_R_tor = m_pSeriesTor_waist->count();
                qreal tor_R = qRadiansToDegrees(m_Motors[joint]->currentTor);
                m_pSeriesTor_R->append(x_R_tor, tor_R);

                if (x_R_tor >100) {

                    m_pSeriesTor_R->clear();
                }
            }
        }
    }
}
void Drum::updateUI()
{
    // 기타 UI 업데이트 관련 함수 호출
    setChartValue();
    setMotorInfos();
    setCanState();
    setMainState();
    setMotorStatus();
}


void Drum::on_pushButton_all_home_clicked()
{
    m_pHomeManager->m_MotorName = "all";
}


void Drum::on_pushButton_home_clicked()
{
    m_pHomeManager->m_MotorName = ui->comboBox_home_joint->currentText().toStdString();
}


void Drum::on_tabWidget_currentChanged(int index)
{

}







void Drum::on_comboBox_view_pos_currentTextChanged(const QString &name)
{
    m_pSeriesPos_L->clear();
    m_pSeriesPos_R->clear();
    m_pSeriesPos_waist->clear();
}


void Drum::on_comboBox_view_vel_currentTextChanged(const QString &name)
{
    m_pSeriesVel_L->clear();
    m_pSeriesVel_R->clear();
    m_pSeriesVel_waist->clear();
}


void Drum::on_comboBox_view_tor_currentTextChanged(const QString &name)
{
    m_pSeriesTor_L->clear();
    m_pSeriesTor_R->clear();
    m_pSeriesTor_waist->clear();
}


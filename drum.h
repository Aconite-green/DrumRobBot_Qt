#ifndef DRUM_H
#define DRUM_H



#include <QMainWindow>
#include <QtCharts>
#include <QLineSeries>
#include <QTimer>
#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>

#include "./include/motors/Motor.hpp"
#include "./include/managers/PathManager.hpp"
#include "./include/managers/CanManager.hpp"
#include "./include/managers/TestManager.hpp"
#include "./include/managers/HomeManager.hpp"

#include "./include/tasks/DrumRobot.hpp"
#include "./include/tasks/SystemState.hpp"



using namespace std;

QT_BEGIN_NAMESPACE

namespace Ui {
class Drum;
}

QT_END_NAMESPACE

class Drum : public QMainWindow
{
    Q_OBJECT

public:
    Drum(QWidget *parent = nullptr);
    ~Drum();

private slots:
    void setMainState();
    void setMotorInfos();
    void setCanState();
    void setHomeDone();
    void setMotorStatus();
    void setChartValue();

    void on_comboBox_state_demand_currentTextChanged(const QString &arg1);

    void on_pushButton_music_clicked();

    void on_pushButton_start_clicked();



    void on_pushButton_all_home_clicked();

    void on_pushButton_home_clicked();

    void on_tabWidget_currentChanged(int index);

    void on_comboBox_view_pos_currentTextChanged(const QString &arg1);

    void on_comboBox_view_vel_currentTextChanged(const QString &arg1);

    void on_comboBox_view_tor_currentTextChanged(const QString &arg1);

private:
    Ui::Drum *ui;

    SystemState m_SystemState;
    std::map<std::string, std::shared_ptr<GenericMotor>> m_Motors;

    CanManager *m_pCanManager;
    PathManager *m_pPathManager;

    TestManager *m_pTestManager;
    HomeManager *m_pHomeManager;

    DrumRobot *m_pDrumRobot;


    std::thread m_StateThread;
    std::thread m_SendThread;
    std::thread m_ReceiveThread;

    QTimer *m_pTimerUI;
    QLineSeries *m_pSeriesPos_L;
    QLineSeries *m_pSeriesPos_R;
    QLineSeries *m_pSeriesPos_waist;
    QChart *m_pChartPos;

    QLineSeries *m_pSeriesVel_L;
    QLineSeries *m_pSeriesVel_R;
    QLineSeries *m_pSeriesVel_waist;
    QChart *m_pChartVel;

    QLineSeries *m_pSeriesTor_L;
    QLineSeries *m_pSeriesTor_R;
    QLineSeries *m_pSeriesTor_waist;
    QChart *m_pChartTor;



    QValueAxis *m_pAxisPosX;
    QValueAxis *m_pAxisPosY;

    QValueAxis *m_pAxisVelX;
    QValueAxis *m_pAxisVelY;

    QValueAxis *m_pAxisTorX;
    QValueAxis *m_pAxisTorY;



private:
    void init();
    void initConnections();
    void initUi();
    void initChart();
    void updateUI();

};
#endif // DRUM_H

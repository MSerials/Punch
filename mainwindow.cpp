#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "settingdialog.h"
#include <iostream>
#include <QTouchEvent>
#include <QCoreApplication>
#include <QDialog>
#include <QFileDialog>
#include <list>
#include "modelsetdialog.h"
#include "global.h"
#include "opencv.hpp"
#include "mediator.h"
#include "excv.h"
#include <QMessageBox>
#include <QTextCodec>
#include "cryptdialog.h"

MainWindow *pMainWin = nullptr;
int counter = 0;

void show_counter()
{
    QString str = QString::fromLocal8Bit("冲压次数:") + QString::number(counter);
    if(nullptr != pMainWin)
        pMainWin->ShowCounter(str);
    counter++;
}

void MainWindow::ShowCounter(QString str)
{
    ui->label_Qty->setText(str);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  //  QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8")); // 改为GBK编码
    ui->setupUi(this);
    pMainWin = this;
#ifdef SHOW_FULL_SCREEN
    showFullScreen();
#endif
    setEnabled(true);
    setAttribute(Qt::WA_AcceptTouchEvents);
    ui->pushButton_Up->setAttribute(Qt::WA_AcceptTouchEvents);
    ui->pushButton_Up->installEventFilter(this);
    ui->pushButton_Down->setAttribute(Qt::WA_AcceptTouchEvents);
    ui->pushButton_Down->installEventFilter(this);
    ui->pushButton_Left->setAttribute(Qt::WA_AcceptTouchEvents);
    ui->pushButton_Left->installEventFilter(this);
    ui->pushButton_Right->setAttribute(Qt::WA_AcceptTouchEvents);
    ui->pushButton_Right->installEventFilter(this);

    int nWndWidth = ui->widget->width();
    int nWndHeight= ui->widget->height();
    HalconCpp::SetCheck("~father");
    HalconCpp::OpenWindow(0, 0, nWndWidth, nWndHeight, (Hlong)ui->widget->winId(), "visible", "", &    Mediator::GetIns()->MainWindowDispHd);
    HalconCpp::SetCheck("father");
    Mediator::GetIns()->UpdateHistory = UpdateHistoryInfo;
    Mediator::GetIns()->show_StartButton = show_State_Ex;
    Mediator::GetIns()->show_Counter = show_counter;
    show_counter();
}

void MainWindow::Init(const char* pName)
{
    ProgmaName = std::string(pName);
    std::cout <<"ProgramName is "<<ProgmaName<<std::endl;

    QDir dir;
    if(!dir.exists(CABLI)) dir.mkdir(CABLI);
    if(!dir.exists(CABLI)) dir.mkdir(MODEL);
    if(!dir.exists(CABLI)) dir.mkdir(INI);

    //防止程序多开
    HANDLE m_hMutex  =  ::CreateMutexW(NULL, FALSE,  L"PUNCH_____2019.1.5" );
    //  检查错误代码
    if  (GetLastError()  ==  ERROR_ALREADY_EXISTS)  {
        QMessageBox::warning(this,QString::fromLocal8Bit("Error"),QString::fromLocal8Bit("请勿多开程序!如果频繁出现，请重启电脑"));
      //  如果已有互斥量存在则释放句柄并复位互斥量
     CloseHandle(m_hMutex);
     m_hMutex  =  NULL;
     exit(0);
    }

    char cVersion[256] = {0};
    CryptDialog CryptDlg(cVersion);
    if(false == CryptDlg.Init())
    CryptDlg.exec();

    //初始化参数
    Preference::GetIns()->prj->SetFilePos(QString(PRJ_PATH));
    using namespace cv;
    FileStorage fs2("Cabli.yml", FileStorage::READ);
    fs2["cameraMatrix"] >> Mediator::GetIns()->m_cameraMatrix;
    fs2["distCoeffs"] >> Mediator::GetIns()->m_distCoeffs;
    std::cout<<"旋转" << Mediator::GetIns()->m_cameraMatrix<<std::endl;
    std::cout<<"畸变" << Mediator::GetIns()->m_distCoeffs<<std::endl;


    QString Info;
    int Camera_qty = MSerialsCamera::init_camera();
    if(Camera_qty < 1){
        Info += QString::fromLocal8Bit("没有发现相机");
    }
    else
    {
        MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED,0.0,0,Mediator::GetIns()->m_cameraMatrix,Mediator::GetIns()->m_distCoeffs);
    }

#ifndef NO_MOTION
    //初始化板卡
    int Card_Qty = motion::GetIns()->init();
    if(Card_Qty < 1) {
        Info += QString::fromLocal8Bit(" 没有发现控制卡");
    }
    else
    {
        motion::GetIns()->CurrentCard()->SetNegLimit(X_AXIS_MOTOR, X_AXIS_LIMIT);
    }
#endif

    if(!Info.isEmpty())
        QMessageBox::warning(NULL,QString("Error"),Info);

    Mediator::GetIns()->UpdateMessage("启动程序");

    //初始化相机
    Mediator::GetIns()->Load_Model(Preference::GetIns()->prj->Model_Name.toLocal8Bit().toStdString().data(),Mediator::GetIns()->MainWindowDispHd,false);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_Quit_clicked()
{
    close();
}

void MainWindow::on_pushButton_ParamSet_clicked()
{
    //判断是否运动状态
    if(RUN == (Mediator::GetIns()->GetState()&RUN))
    {
        UpdateHistory("请停止后设置");
        return;
    }

    static SettingDialog * SetDlg = nullptr;
    if(nullptr != SetDlg)
    {
        delete SetDlg;
        SetDlg = nullptr;
    }

    SetDlg = new SettingDialog();

    SetDlg->show();
    SetDlg->exec();
}

void MainWindow::on_pushButton_ModelSet_clicked()
{
    //判断是否运动状态
    if(RUN == (Mediator::GetIns()->GetState()&RUN))
    {
        UpdateHistory("请停止后设置");
        return;
    }
    static ModelSetDialog *ModelSetDlg  = nullptr;
    if(nullptr != ModelSetDlg)
    {
        delete ModelSetDlg;
        ModelSetDlg = nullptr;
    }

    ModelSetDlg = new ModelSetDialog();

    ModelSetDlg->show();
    ModelSetDlg->exec();
}


bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    static bool isTouched = false;
    switch (event->type()) {
    case QEvent::TouchBegin:
        isTouched = true;
        if(obj == ui->pushButton_Up)
        {
            Mediator::GetIns()->MachineOp(MOVE_UP);
        }
        else if(obj == ui->pushButton_Down)
        {
            Mediator::GetIns()->MachineOp(MOVE_DOWN);
        }
        else if(obj == ui->pushButton_Left)
        {
            Mediator::GetIns()->MachineOp(MOVE_LEFT);
        }
        else if(obj == ui->pushButton_Right)
        {
           Mediator::GetIns()->MachineOp(MOVE_RIGHT);
        }
        break;
    case QEvent::MouseButtonPress:
        if(isTouched) break;
        if(obj == ui->pushButton_Up)
        {
            Mediator::GetIns()->MachineOp(MOVE_UP);
        }
        else if(obj == ui->pushButton_Down)
        {
            Mediator::GetIns()->MachineOp(MOVE_DOWN);
        }
        else if(obj == ui->pushButton_Left)
        {
            Mediator::GetIns()->MachineOp(MOVE_LEFT);
        }
        else if(obj == ui->pushButton_Right)
        {
           Mediator::GetIns()->MachineOp(MOVE_RIGHT);
        }
        break;
    case QEvent::TouchEnd:
    case QEvent::MouseButtonRelease:
        if(obj == ui->pushButton_Up || obj == ui->pushButton_Down || obj == ui->pushButton_Left || obj == ui->pushButton_Right)
        {
            Mediator::GetIns()->MachineOp(MOVE_STOP);
        }
        isTouched = false;
        break;
    default:
        break;
    }
    return QMainWindow::eventFilter(obj,event);
}

void MainWindow::MachineOp(int Sel, int SecondSel)
{
    SecondSel = 1;
    printf("select %d\n",Sel);
}

void MainWindow::show_State_Ex()
{
    pMainWin->show_State();
}

void MainWindow::show_State()
{
        long long state = Mediator::GetIns()->GetState();
        if(PAUSE == (state&PAUSE))
        {
            ui->pushButton_Run->setText(QString::fromLocal8Bit("运行"));
        }
        else if(STOP == (state&STOP))
        {
            ui->pushButton_Run->setText(QString::fromLocal8Bit("运行"));
        }
        else if(RUN == (state&RUN))
        {
            ui->pushButton_Run->setText(QString::fromLocal8Bit("暂停"));
        }
}

void MainWindow::on_pushButton_Run_clicked()
{
    if(RUN == (Mediator::GetIns()->GetState()&RUN))
    {
        Mediator::GetIns()->MachineOp(MACHINE_PAUSE);
    }
    else{
        Mediator::GetIns()->MachineOp(MACHINE_START);
    }
    show_State();
}

void MainWindow::on_pushButton_Stop_2_clicked()
{
    Mediator::GetIns()->MachineOp(MACHINE_STOP);
    ui->pushButton_Run->setText(QString::fromLocal8Bit("运行"));
    return;
    if(RUN == (Mediator::GetIns()->GetState()&RUN))
    {
        Mediator::GetIns()->MachineOp(MACHINE_PAUSE);
        ui->pushButton_Run->setText(QString::fromLocal8Bit("暂停"));
    }
    else{
        Mediator::GetIns()->MachineOp(MACHINE_STOP);
        ui->pushButton_Run->setText(QString::fromLocal8Bit("运行"));
    }
}

void MainWindow::on_pushButton_clear_clicked()
{
    counter = 0;
    show_counter();
}

void  MainWindow::UpdateHistoryInfo(std::string Info)
{
   pMainWin->UpdateHistory(Info);
}

void  MainWindow::UpdateHistory(std::string Info)
{
    static std::list<std::string> InfoMations;
    InfoMations.push_back(Info);
    while(InfoMations.size()>10) InfoMations.pop_front();
    std::string Msg;
    for(auto I:InfoMations){Msg += I+"\n";}
    ui->LabelHis->setText(QString::fromLocal8Bit(Msg.c_str()));
}

void MainWindow::on_pushButton_clicked()
{
    Mediator::GetIns()->MachineOp(MACHINE_ORIGIN);
}

void MainWindow::on_pushButton_Punch_clicked()
{
    Mediator::GetIns()->MachineOp(MACHINE_PUNCH);
}

void MainWindow::on_pushButton_Stop_clicked()
{
    printf("ProcessState is %d  PunchState is %d GetPointState is %d\n",Mediator::GetIns()->evt_Process.State(),Mediator::GetIns()->evt_Punch.State(),Mediator::GetIns()->evt_GetPoint.State());
}

void MainWindow::on_pushButton_Press_clicked()
{
    Mediator::GetIns()->MachineOp(MACHINE_PUNCH);
    Sleep(200);
    Mediator::GetIns()->MachineOp(MACHINE_SEPRA);
}

void MainWindow::on_pushButton_Seperate_clicked()
{
    Mediator::GetIns()->MachineOp(MACHINE_SEPRA);
}

#include "modelsetdialog.h"
#include "ui_modelsetdialog.h"
#include "mediator.h"
#include "global.h"

#include "excv.h"
#include <QMessageBox>
#include "global.h"
#include <process.h>
#include <QDesktopServices>
#include <QUrl>
//#include <QFileDialog>

bool isGrab = false;
bool isGrabing = false;

static void __stdcall DispMatImage_Ex(cv::Mat InputArray, void * pWnd = nullptr, bool Inv = false){
    try
    {
        HalconCpp::HObject hobj;
        Excv::MatToHObj(InputArray,hobj);
        Excv::h_disp_obj(hobj,Mediator::GetIns()->ModelDisp);
    }catch(cv::Exception ex)
    {

    }
}


void ModelSetDialog::CamSnap()
{
    printf("get image from camera\n");
    //两张图片比较是否掉线
    cv::Mat Image_Try   = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED,CAMERA_ANGLE).clone();
    Snap_Image       = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED,CAMERA_ANGLE).clone();
    if(true == MSerialsCamera::isEqual(Snap_Image,Image_Try))
    {
#if 1
        for (int i = 0;;i++)
        {
            Sleep(100);
            if(MSerialsCamera::init_camera() > 0) {
                    Snap_Image       = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED,CAMERA_ANGLE).clone();
                    break;
            }
            else if (i>3)
            {
                QString fileName = QFileDialog::getOpenFileName(NULL,
                    tr("Open Image"), "",
                    tr("All Files (*)"));
                cv::Mat Image = cv::imread(fileName.toLocal8Bit().data());
                if(Image.empty())   return;
                Snap_Image = Image.clone();
                CvCvtColor(Image,Snap_Image,CV_BGR2GRAY);
                cv::resize(Snap_Image, Image, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 0.0,0.0, cv::INTER_CUBIC);
                MSerialsCamera::VoidImage() = Image.clone();
                break;
            }
        }
#endif
    }
    try{
    HalconCpp::HObject Hobj,ROI;
    GenRectangle1(&ROI,CHECK_R1,CHECK_C1,CHECK_R2,CHECK_C2);
    Excv::MatToHObj(Snap_Image,Hobj);
    Excv::h_disp_obj(Hobj,Mediator::GetIns()->ModelDisp);
    SetDraw(Mediator::GetIns()->ModelDisp,"margin");
    SetColor(Mediator::GetIns()->ModelDisp,"yellow");
    DispObj(ROI,Mediator::GetIns()->ModelDisp);
    }catch(cv::Exception ex)
    {
        HalconCpp::SetTposition(Mediator::GetIns()->ModelDisp,10,0);
        HalconCpp::SetColor(Mediator::GetIns()->ModelDisp,"red");
        HalconCpp::WriteString(Mediator::GetIns()->ModelDisp,ex.what());
    }
}

ModelSetDialog::ModelSetDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelSetDialog)
{
    ui->setupUi(this);
#ifdef SHOW_FULL_SCREEN
    showFullScreen();
#endif
    //isCabling = false;
    int nWndWidth = ui->widget->width();
    int nWndHeight= ui->widget->height();
    HalconCpp::SetCheck("~father");
    HalconCpp::OpenWindow(0, 0, nWndWidth, nWndHeight, (Hlong)ui->widget->winId(), "visible", "", &    Mediator::GetIns()->ModelDisp);
    HalconCpp::SetCheck("father");

    //bug??设计师的信号槽不能用了？
    connect(ui->pushButton_Snap,&QPushButton::clicked,[=](){CamSnap();});
    connect(ui->pushButton_OSK,&QPushButton::clicked,[=](){ QDesktopServices::openUrl(QUrl("osk.exe", QUrl::TolerantMode));});
    connect(ui->pushButton_CloseSet,&QPushButton::clicked,[=](){isGrab = false;close();});
    ui->lineEdit_Di->setText(QString::number(35));
    connect(ui->pushButton_GenCircle,&QPushButton::clicked,[=](){on_pushButton_GenCircle_clicked();});
    ui->lineEdit_RectW->setText(QString::number(20));
    ui->lineEdit_RectH->setText(QString::number(20));
    connect(ui->pushButton_GenRect,&QPushButton::clicked,[=](){on_pushButton_GenRect_clicked();});
    connect(ui->pushButton_GetModel,&QPushButton::clicked,[=](){QString fileName = QFileDialog::getOpenFileName(NULL,tr("Open Image"), "Model",tr("All Files (*)"));
        Mediator::GetIns()->Load_Model(fileName.toLocal8Bit().data(),Mediator::GetIns()->ModelDisp,true);
    });
    connect(ui->pushButton_SetCheckArea,&QPushButton::clicked,[=](){
        using namespace HalconCpp;
        try{
        if(isDraw) return;
        HObject ROI;
        Snap_Image = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED,CAMERA_ANGLE);        HalconCpp::HObject Hobj;
        DispMatImage_Ex(Snap_Image);
        SetColor(Mediator::GetIns()->ModelDisp,"yellow");
        SetDraw(Mediator::GetIns()->ModelDisp,"margin");
        isDraw = true;
        HTuple R1,R2,C1,C2;
        DrawRectangle1(Mediator::GetIns()->ModelDisp, &R1,&C1,&R2,&C2);
        CHECK_R1 = R1.D();CHECK_C1 = C1.D();CHECK_R2 = R2.D();CHECK_C2 = C2.D();
        GenRectangle1(&ROI,CHECK_R1,CHECK_C1,CHECK_R2,CHECK_C2);
        DispObj(ROI,Mediator::GetIns()->ModelDisp);
        isDraw = false;
        Preference::GetIns()->prj->WriteSettings();
        }
        catch (HalconCpp::HException except)
        {
            HalconCpp::HTuple Except;
            except.ToHTuple(&Except);
            return;
        }
        catch(cv::Exception ex)
        {
            printf("ex %s\n",ex.what());
            return;
        }
    });

    connect(ui->pushButton_DrawModelArea,&QPushButton::clicked,[=](){
        using namespace HalconCpp;
        try{
        if(isDraw) return;
        HObject ROI;
        DispMatImage_Ex(Snap_Image);
        SetColor(Mediator::GetIns()->ModelDisp,"cyan");
        SetDraw(Mediator::GetIns()->ModelDisp,"margin");
        isDraw = true;
        HTuple R1,R2,C1,C2;
        DrawRectangle1(Mediator::GetIns()->ModelDisp, &R1,&C1,&R2,&C2);
        MODEL_R1 = R1.D();MODEL_C1 = C1.D();MODEL_R2 = R2.D();MODEL_C2 = C2.D();
        GenRectangle1(&ROI,R1,C1,R2,C2);
        DispObj(ROI,Mediator::GetIns()->ModelDisp);
        isDraw = false;
        Preference::GetIns()->prj->WriteSettings();
        }
        catch (HalconCpp::HException except)
        {
            HTuple ErrorInfo;
            except.ToHTuple(&ErrorInfo);
            isDraw = false;
            return;
        }
        catch(cv::Exception ex)
        {
            isDraw = false;
            HalconCpp::SetColor(Mediator::GetIns()->ModelDisp,"red");
            HalconCpp::SetTposition(Mediator::GetIns()->ModelDisp,1,1);
            HalconCpp::WriteString(Mediator::GetIns()->ModelDisp,ex.what());
        }
    });

     connect(ui->pushButton_SetModel,&QPushButton::clicked,[=](){
         try{
                HalconCpp::HObject Hobj,ROI;
                DispMatImage_Ex(Snap_Image);
                Excv::MatToHObj(Snap_Image,Hobj);
                cv::Rect Rt(MODEL_C1,MODEL_R1,MODEL_C2-MODEL_C1,MODEL_R2-MODEL_R1);
                cv::Mat ImageReduced,thresHold;
                Snap_Image(Rt).copyTo(ImageReduced);
                cv::threshold(ImageReduced,thresHold,Get_Obj_Threshold,255,CV_THRESH_BINARY);
                std::string model_name = Excv::cv_write_image(thresHold, "Model", "Model",true);
                Mediator::GetIns()->Load_Model(model_name,Mediator::GetIns()->ModelDisp,true);
         }
         catch(cv::Exception ex)
         {
               HalconCpp::SetColor(Mediator::GetIns()->ModelDisp,"red");
               HalconCpp::SetTposition(Mediator::GetIns()->ModelDisp,1,1);
               HalconCpp::WriteString(Mediator::GetIns()->ModelDisp,ex.what());
         }
     });

     connect(ui->pushButton_Grab,&QPushButton::pressed,[=](){on_pushButton_Grab_clicked();});
     isOpend = true;

     for(int i =  LINES_HORIZONTAL_AI ; i <= DOUBLE_VERTICAL ; i++)
     ui->comboBox_LineMethod->addItem(QString::fromLocal8Bit(_global::LineMethod()[i].c_str()));

     connect(ui->comboBox_LineMethod,&QComboBox::currentTextChanged,[=](QString Str){
         int Idx = ui->comboBox_LineMethod->currentIndex();
         LINE_METHOD = Idx;
         std::cout << Str.toStdString() <<std::endl;
     });
}

ModelSetDialog::~ModelSetDialog()
{
    delete ui;
}



void ModelSetDialog::on_pushButton_clicked()
{
    close();
}

bool ModelSetDialog::Load_Model(std::string file_name)
{
    file_name = "cpp";
  return true;
}

void ModelSetDialog::on_pushButton_GenCircle_clicked()
{
    double Diameter = ui->lineEdit_Di->text().toDouble();
    double radius = Diameter/2;
    if (radius < 1)
    {
        QMessageBox::information(NULL,"Error",QString::fromLocal8Bit("尺寸太小，请检查输入是否正确？"));
        return;
    }
    cv::Point pix;
    Excv::mm_to_pix(cv::Point2d(2*radius, 2*radius), pix);
    int image_h = static_cast<int>(pix.x);
    int image_w = image_h;
    cv::Mat Model(image_h+3, image_w+3, CV_8UC1, cv::Scalar(0));
    cv::Point center(image_w / 2+1, image_h / 2+1);
    cv::circle(Model, center, static_cast<int>(image_w / 2),cv::Scalar(255),-1);
    char model_name[256] = { 0 };
    sprintf(model_name, "直径_%5.2fmm", Diameter);
    std::string _model_name = Excv::cv_write_image(Model, "Model", model_name);
    Mediator::GetIns()->Load_Model(_model_name,Mediator::GetIns()->ModelDisp);
}

void ModelSetDialog::on_pushButton_GenRect_clicked()
{
    double w = ui->lineEdit_RectW->text().toDouble();
    double h = ui->lineEdit_RectH->text().toDouble();
    if (w < 2 || h<2)
    {
        QMessageBox::information(NULL,"Error",QString::fromLocal8Bit("尺寸太小，请检查输入是否正确？"));
        return;
    }
    cv::Point pix;
    Excv::mm_to_pix(cv::Point2d(w, h), pix);
    int image_w = static_cast<int>(pix.x);
    int image_h = static_cast<int>(pix.y);
    cv::Rect Rt(1,1,image_w,image_h);
    cv::Mat Model(image_h+3, image_w+3, CV_8UC1, cv::Scalar(0));
    cv::rectangle(Model,Rt,cv::Scalar(255,255,255),-1);
    char model_name[256] = { 0 };
    sprintf_s(model_name, "宽%5.2fmm-高%5.2fmm",w,h);
    std::string _model_name = Excv::cv_write_image(Model, "Model", model_name);
    Mediator::GetIns()->Load_Model(_model_name,Mediator::GetIns()->ModelDisp);
}



void ModelSetDialog::on_pushButton_SaveImage_clicked()
{
    try{
        cv::imwrite("a.png",Snap_Image);
        cv::Mat Gray;
        MSerialsCamera::CvtColor(Snap_Image,Gray);
        cv::imwrite("b.png",Gray);
    }catch(cv::Exception ex)
    {

    }
}

unsigned int __stdcall Grab(void* p)
{
    if(isGrabing) return 1;
    isGrabing = true;
    for(;isGrab;)
    {
        cv::Mat Snap = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED,CAMERA_ANGLE).clone();
        DispMatImage_Ex(Snap);
    }
    isGrabing = false;
    return 0;
}
void ModelSetDialog::on_pushButton_Grab_clicked()
{
    if(isGrab == true)
    {
       isGrab = false;
       isDraw = false;
       ui->pushButton_Grab->setText(QString::fromLocal8Bit("连续截图"));
       printf("is grab\n");
    }
    else
    {
       if(isGrabing)
       {
            return;
       }
       isGrab = true;
       isDraw = true;
       printf("is not grab\n");
       ui->pushButton_Grab->setText(QString::fromLocal8Bit("停止截图"));
       (HANDLE)_beginthreadex(NULL, 0, Grab, this, 0, NULL);
    }
}






void ModelSetDialog::on_pushButton_OSK_clicked()
{

}

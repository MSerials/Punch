﻿#ifndef COMMON_H
#define COMMON_H

#include <atomic>
#include "opencv.hpp"


#if !defined(_____PUNCH_COMMON_20181014_____)
#define _____PUNCH_COMMON_20181014_____

//#define SHOW
//#define TEST
//#define SHOW_CONSOLE

#ifndef METHOD_SELECT
#define METHOD_SELECT

#define LINES_HORIZONTAL_AI			0
#define LINES_HORIZONTAL			1
#define	LINES_VERTICAL_AI			2
#define LINES_VERTICAL				3
#define DOUBLE_HORIZONTAL			4
#define DOUBLE_VERTICAL				5
#define	LINES_VERTICAL_MARTIX		6
#define LINES_HORIZONTAL_MARTIX		7

#endif


#define PUNCH_VERSION														"18.11.0"

enum tagSystemState { STOP, PAUSE, RUN, EMSTOP, TESTING, GETTING_MODEL, GETTING_BACK };

struct Control_Var {
public:
    Control_Var() {
        ROI = cv::Rect(0, 0, 0, 0);
        Cal_ROI = cv::Rect(0, 0, 0, 0);
        image_threshold = 192.0;
    }

    Control_Var(const Control_Var &Ctrl_Var) {
        system_state = Ctrl_Var.system_state;
        error_code = Ctrl_Var.error_code;
        Lines_Method = Ctrl_Var.Lines_Method;
        image_threshold = Ctrl_Var.image_threshold;
        model_distance = Ctrl_Var.model_distance;
        margin_to_model = Ctrl_Var.margin_to_model;
        Pos_Acc = Ctrl_Var.Pos_Acc;
        HWND_History = Ctrl_Var.HWND_History;
        pWnd = Ctrl_Var.pWnd;
        isDisplay = Ctrl_Var.isDisplay;
        pos_models = Ctrl_Var.pos_models;
        ModelsPostion = Ctrl_Var.ModelsPostion;
        Cal_ModelsPostion = Ctrl_Var.Cal_ModelsPostion;
        CurrentYAxisPostion = Ctrl_Var.CurrentYAxisPostion;
        CurrentXAxisPostion = Ctrl_Var.CurrentXAxisPostion;
        MovingForwardPuls = Ctrl_Var.MovingForwardPuls;
        dMovingForwardPuls = Ctrl_Var.dMovingForwardPuls;
        ModelContours = Ctrl_Var.ModelContours;
        LoadModelDispContours = Ctrl_Var.LoadModelDispContours;
        DoubleContours = Ctrl_Var.DoubleContours;
        ROI = Ctrl_Var.ROI;
        Cal_ROI = Ctrl_Var.Cal_ROI;
        w_axis = Ctrl_Var.w_axis;
        h_axis = Ctrl_Var.h_axis;
        last_y_pos = Ctrl_Var.last_y_pos;
        last_y_bottom_pos = Ctrl_Var.last_y_bottom_pos;
        Tale_Pix = Ctrl_Var.Tale_Pix;
        DispMatImage = Ctrl_Var.DispMatImage;
        MMessageBox = Ctrl_Var.MMessageBox;
        Buffer = Ctrl_Var.Buffer;
        error_infomation = Ctrl_Var.error_code;
    }

    tagSystemState SystemState = tagSystemState::STOP;
    //系统暂停状态变量
    int system_state = 0;

    //系统错误变量
    int error_code = 0;

    //排列方式
    int Lines_Method = 0;

    //全局二值化变量
    double image_threshold = 192.0;

    //摸具像素距离
    double model_distance = 1.0;

    //边界到模具距离
    double margin_to_model = 1.0;

    //还剩多少个脉冲时候就冲下来
    long Pos_Acc = 0;

    //获取主窗口History的句柄
    void *HWND_History = nullptr;

    void* pWnd = nullptr;

    //是否显示运行图片
    bool isDisplay = false;

    //参数，模具实际的像素坐标
    std::vector<std::vector<cv::Point>> pos_models;

    //参数，获得的实际磨具位置,单位分是puls,和图像处理的得到的位置，单位是pix
    //2l 是64位的，能够存储更多的字节
    std::list<std::vector<cv::Point2l>> ModelsPostion;
    //进行复用，在计算前变成模具位置占用图片的信息，在计算完毕后就是计算出来的模具位置信息。
    std::list<std::vector<cv::Point>> Cal_ModelsPostion;

    //当前Y轴心位置，用于记录暂停信息
    int CurrentYAxisPostion = 0, CurrentXAxisPostion = 0, MovingForwardPuls = 0;
    double dMovingForwardPuls = 0.0;

    //参数，获得的模具轮廓
    std::vector<std::vector<cv::Point>> ModelContours,LoadModelDispContours,DoubleContours;

    //ROI相机检测的范围， Cal_ROI，图片被扩展的范围
    cv::Rect ROI, Cal_ROI;

    std::vector<int> w_axis;
    //强制性h方向布局开始 -1代表取消掉这个选项
    std::vector<int> h_axis;
    //
    int last_y_pos = -1;

    int last_y_bottom_pos = -1;

    //传递给图像知道最后遍历到最后一排的参数，保证计算正确
    int Tale_Pix = 0;

    //因为dll的限制，这个用于回调进行显示,参数含义分别是 mat cstatic pWnd 和 是否反转显示
    typedef void (__stdcall *DispMat)(cv::Mat, void*, bool);
    DispMat DispMatImage = nullptr;

    //回調，讓機標準C++調用到MEssageBox
    typedef int (__stdcall *MsgBox)(const char*, unsigned int);
    MsgBox MMessageBox = nullptr;

    cv::Mat Buffer;
    std::string error_infomation;
    void reset() {
        h_axis.clear();
        w_axis.clear();
        ModelsPostion.clear();
        if(!Buffer.empty()) Buffer.release();
        Cal_ModelsPostion.clear();
        DoubleContours.clear();
    }
};



#endif


#endif // COMMON_H

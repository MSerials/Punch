#include "mainwindow.h"
#include <QApplication>
#include "global.h"
#include <iostream>
#include <cryptdialog.h>

//所有的lib链接库在这里,不知道什么bug，不能使用相对路径
#pragma comment(lib,"d:/Users/Lux/Desktop/ProjectFile/third_party/yanwei/inc/IMCnet.lib")
#pragma comment(lib,"d:/Users/Lux/Desktop/ProjectFile/third_party/opencv3.4.0/x64/vc14/lib/opencv_world340.lib")
#pragma comment(lib,"d:/Users/Lux/Desktop/ProjectFile/third_party\\halcon12\\halconcpp.lib")
#pragma comment(lib,"d:\\Users\\Lux\\Desktop\\ProjectFile\\ChongChuang12\\x64\\Release\\MSerialsCV.lib")
#pragma comment(lib,"d:\\Users\\Lux\\Desktop\\ProjectFile\\ChongChuang12\\x64\\Release\\MSerialsCamera.lib")
#pragma comment(lib,"d:/Users/Lux/Desktop/ProjectFile/third_party/crypt/vc14x64/cryptlib.lib")




int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //char cVersion[256] = {0};
    CryptDialog CryptDlg;
    if(false == CryptDlg.Init())
    CryptDlg.exec();

    MainWindow w;
    w.Init(std::string(argv[0]).c_str());
    w.show();
    return a.exec();
}

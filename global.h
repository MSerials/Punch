#include "debuginfo.h"
#include "MSerialsCV.h"
#include "cameras.h"
#include "preferences.h"
#include <windows.h>
#include <mutex>
#include "MSerialsCore/motion.h"
#include "vector"
#include "list"
#include <QFileDialog>
#include <QMessageBox>
#include <QProcess>
#include <omp.h>
#include <QDir>
#include <unordered_map>


#ifndef GLOBAL_H
#define GLOBAL_H

#define CABLI "Cabli"
#define MODEL "Model"
#define INI   "prj"

//0 2048x1516
//1 1600x1200
//2 1280x1024
//3 1024x768
#define MODE_INDEX      2
#define IMAGE_WIDTH     1024
#define IMAGE_HEIGHT    768


#define ERROR_ADMIN_	-1
#define ERROR_OK_		0
#define ERROR_AUTHOR_	3
#define ERROR_TIMEOUT_	4

#define _VERSION       "20190212"
#define __VERSION      L"20190122"

class MEvent
{
private:
    std::mutex mtx;
    HANDLE evt;
    int state;
public:
    MEvent():
        state(0)
    {
        evt = CreateEventW(FALSE,TRUE,FALSE,NULL);
        ::ResetEvent(evt);
    }
    const HANDLE &get(){return evt;}
    int SetEvent() {std::lock_guard<std::mutex> lck(mtx);::SetEvent(evt);return (state = 1);}
    int ResetEvent(){std::lock_guard<std::mutex> lck(mtx);::ResetEvent(evt);return (state = 0);}
    int &State() { return state; }
};

class _global{
public:
    _global(){
        LineMethod()[LINES_HORIZONTAL_AI] = "智能横向排列";
        LineMethod()[LINES_HORIZONTAL] = "横向排列";
        LineMethod()[LINES_VERTICAL_AI] = "智能纵向排列";
        LineMethod()[LINES_VERTICAL] = "纵向排列";
        LineMethod()[DOUBLE_HORIZONTAL] = "左右双面";
        LineMethod()[DOUBLE_VERTICAL] = "上下双面";
        printf("global\n");
    }
    static long GetThisYearDay(int year, int month, int day)
    {
        int counterOfOddYear = (year / 4);
        if (!(year % 4)) {
            counterOfOddYear--;
            long allday = (year - 1) * 365 + counterOfOddYear;
            switch (month)
            {
            case 1: return (day + allday);
            case 2: return (day + allday + 31);
            case 3: return (day + allday + 60);
            case 4: return (day + allday + 91);
            case 5: return (day + allday + 121);
            case 6: return (day + allday + 152);
            case 7: return (day + allday + 182);
            case 8: return (day + allday + 213);
            case 9: return (day + allday + 244);
            case 10: return (day + allday + 274);
            case 11: return (day + allday + 305);
            case 12: return (day + allday + 335);
            default:return 0;
            }
        }
        else
        {
            long allday = (year) * 365 + counterOfOddYear;
            switch (month)
            {
            case 1: return (day + allday);
            case 2: return (day + allday + 31);
            case 3: return (day + allday + 59);
            case 4: return (day + allday + 90);
            case 5: return (day + allday + 120);
            case 6: return (day + allday + 151);
            case 7: return (day + allday + 181);
            case 8: return (day + allday + 212);
            case 9: return (day + allday + 243);
            case 10: return (day + allday + 273);
            case 11: return (day + allday + 304);
            case 12: return (day + allday + 334);
            default:return 0;
            }

        }
    }

    static std::unordered_map<int,std::string> &LineMethod(){
        static std::unordered_map<int,std::string> LM;
        static bool isOK = false;
        if(!isOK)
        {
            isOK = true;
            LineMethod()[LINES_HORIZONTAL_AI] = "智能横向排列";
            LineMethod()[LINES_HORIZONTAL] = "横向排列";
            LineMethod()[LINES_VERTICAL_AI] = "智能纵向排列";
            LineMethod()[LINES_VERTICAL] = "纵向排列";
            LineMethod()[DOUBLE_HORIZONTAL] = "左右双面";
            LineMethod()[DOUBLE_VERTICAL] = "上下双面";
        }
        return LM;
    }

    static QString GetBoardInfo()
    {
    QString cpu_id = "";
    QProcess p(0);
    p.start("wmic BaseBoard get SerialNumber");
    p.waitForStarted();
    p.waitForFinished();
    cpu_id = QString::fromLocal8Bit(p.readAllStandardOutput());
    cpu_id = cpu_id.remove("SerialNumber").trimmed();
    return cpu_id;
    }

    static void RegSetValue(QString Value,QString Key = "HKEY_CURRENT_USER\\Software\\MicroSoftReg")
    {
        QSettings settings(Key,QSettings::NativeFormat);
        settings.setValue("T", Value);//写
    }

    static QString RegGetValue(QString Key = "HKEY_CURRENT_USER\\Software\\MicroSoftReg")
    {
        QSettings settings(Key,QSettings::NativeFormat);
        return settings.value("T").toString();
    }

    static void SChange(QString &s)
    {
        s.replace("A", "0");
        s.replace("B", "1");
        s.replace("C", "2");
        s.replace("D", "3");
        s.replace("E", "4");
        s.replace("F", "5");
        s.replace("G", "6");
        s.replace("H", "7");
        s.replace("I", "8");
        s.replace("J", "9");
    }


    static void SChangeInv(QString &s)
    {
        s.replace("0", "A");
        s.replace("1", "B");
        s.replace("2", "C");
        s.replace("3", "D");
        s.replace("4", "E");
        s.replace("5", "F");
        s.replace("6", "G");
        s.replace("7", "H");
        s.replace("8", "I");
        s.replace("9", "J");
    }


    static QStringList Split(QString Str,const char* reg = "[Z]+"){
        return Str.split(QRegExp(reg));
    }


    // Certify 对话框
    static int GetDate(long long &TimeStamp, QString &Serials, int &days)
    {
        TimeStamp = -1;
        QString RegVar = RegGetValueW();
        std::vector<QString> Se;
        if(RegVar.isEmpty())
            return ERROR_AUTHOR_;
        for(auto QStr:RegVar.split(QRegExp("[Z]+")))
        {
            Se.push_back(QStr);
        }
                try
                {
                    Serials = Se.at(0);


                    SChange(Se.at(1));
                    days = (Se.at(1)).toInt();

                     SChange(Se.at(2));
                     int year = (Se.at(2)).toInt();

                     SChange(Se.at(3));
                     int month = (Se.at(3)).toInt();

                     SChange(Se.at(4));
                     int day = (Se.at(4)).toInt();

                     bool isDateOK = true;
                     //year 2099
                     if (year < 2000 || year > 2099)
                     {
                         isDateOK = false;
                     }

                     if (month < 1 || month >12)
                     {
                         isDateOK = false;
                     }

                     if (day < 1 || day > 31)
                     {
                         isDateOK = false;
                     }

                     if (!isDateOK)
                     {
                         QMessageBox::information(NULL,"Error",QString::fromLocal8Bit("日期错误"));
                         return ERROR_AUTHOR_;
                     }

                     long GetDays = GetThisYearDay(year, month, day);
                     TimeStamp = GetDays;
                }
                catch (std::out_of_range e)
                {
                    printf_s("注册表分割失败\n");
                    return ERROR_AUTHOR_;
                }

                return ERROR_OK_;
    }

};


#endif // GLOBAL_H

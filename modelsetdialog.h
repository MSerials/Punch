#ifndef MODELSETDIALOG_H
#define MODELSETDIALOG_H

#include <QDialog>
#include <QDebug>
#include "opencv.hpp"

namespace Ui {
class ModelSetDialog;
}

class ModelSetDialog : public QDialog
{
    Q_OBJECT

public:
    void Close_();
    bool isOpend = false;
    explicit ModelSetDialog(QWidget *parent = 0);
    ~ModelSetDialog();
    cv::Mat Snap_Image;

    bool isDraw = false;

    bool Load_Model(std::string file_name);

    void CamSnap();

    void on_pushButton_clicked();

    void on_pushButton_GenCircle_clicked();

    void on_pushButton_GenRect_clicked();

    void on_pushButton_SetModel_clicked();

    void on_pushButton_GetModel_clicked();

    void on_pushButton_DrawModelArea_clicked();

    void on_pushButton_Snap_clicked();

    void on_pushButton_SetCheckArea_clicked();

    void on_pushButton_SaveImage_clicked();

    void on_pushButton_Grab_clicked();

   // void on_pushButton_SaveCalbi_clicked();

private slots:
  //  void on_pushButton_StartCabli_clicked();

private:
    Ui::ModelSetDialog *ui;
};

#endif // MODELSETDIALOG_H

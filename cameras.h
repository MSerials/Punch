#pragma once
#include <iostream>
#include "opencv.hpp"
#include "global.h"
#include "../../MSerialsCamera/MSerialsCam.h"


#ifndef CAMERAS_H
#define CAMERAS_H


namespace MSerialsCamera {

enum{
    IMAGE_ORIGIN = 0, IMAGE_FLIPED = 1
};
    static int number = 0;

    static int init_camera()
    {
        int c = enum_cameras();
        if (c > 0) {
            int  count_res = GetResolutionCount(0, 0);
            int mode_index = 3 < count_res ? 3 : count_res;

            int width = 1024, height = 768;

            width = 2048;
            height  = 1536;
            mode_index = 1;
            SetResolution(width, height, mode_index, 0, 0);
            std::cout << "ccount res:" << count_res << " w:" << width << " h:" << height << std::endl;
        }
        number = c;
        return c;
    }

    static int camera_number()
    {
        return number;
    }



    static void CvtColor(const cv::Mat org, cv::Mat & dst , int sel = CV_BGR2GRAY )
    {
        CV_Assert(!org.empty());
        switch (sel)
        {
        case CV_BGR2GRAY:
            if (CV_8UC1 == org.type())
            {
                dst = org.clone();
            }
            else
            {
                cv::cvtColor(org, dst, CV_BGR2GRAY);
            }
            break;
        case CV_GRAY2BGR:
            if (CV_8UC3 == org.type())
            {
                dst = org.clone();
            }
            else
            {
                cv::cvtColor(org, dst, CV_GRAY2BGR);
            }
        }
    }


    static cv::Mat & VoidImage()
    {
#ifdef NO_MOTION
        static cv::Mat Image = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(20,20,20));
#else
        static cv::Mat Image = cv::Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC1,cv::Scalar(220,220,220));
#endif
        return Image;
    }

    static bool isEqual(cv::Mat &a, cv::Mat &b)
    {
        int aSize = a.cols*a.rows;
        int bSize = b.cols*b.rows;
        if (aSize != bSize)
            return false;
        int len = aSize < bSize ? aSize : bSize;
        uchar *aData = a.data, *bData = b.data;
        for (int i = 0; i < len; i++)
        {
            if (*aData != *bData)
                return false;
            aData++;
            bData++;
        }
        return true;
    }


    class Si
    {
    private:
        Si(){};
    public:
        static Si* GetIns(){static Si si; return& si;}

        cv::Mat map1, map2;        bool ToRemap = false;
    };


    static cv::Mat & GetMachineImage(int sel, double cam_angle =0.0,int Delay = 0, cv::Mat m_cameraMatrix = cv::Mat(), cv::Mat m_distCoeffs = cv::Mat()) {

        //snap已经包含了互斥锁，所以这个函数是阻塞的
        using namespace cv;
        static Mat Camera_Image, Camera_Snap,Camera_Snap_Origin;
        int width = 0, height = 0, ch = 3;
        uchar *data = nullptr;
        Snap(width, height, &data, ch, 0, 0, Delay);
        if (nullptr == data)
        {
            static cv::Mat VoidImage_Ex;
            cv::Mat Rot_Mat = cv::getRotationMatrix2D(cv::Point2f(VoidImage().cols / 2.0, VoidImage().rows / 2.0), cam_angle, 1.0);
            cv::warpAffine(VoidImage(), VoidImage_Ex, Rot_Mat, VoidImage().size());
            return VoidImage_Ex;
        }
        //这个重新赋值w和h的目的是实时矫正
        if (width != Camera_Snap_Origin.cols || height != Camera_Snap_Origin.rows)
        {
            if (3 == ch) Camera_Snap_Origin = cv::Mat(cv::Size(width, height), CV_8UC3);
            else Camera_Snap_Origin = cv::Mat(cv::Size(width, height), CV_8UC1);
        }
        Camera_Snap_Origin.data = data;
        cv::resize(Camera_Snap_Origin, Camera_Image, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 0.0,0.0, cv::INTER_CUBIC);
        if (IMAGE_ORIGIN == sel)
        {
            static cv::Mat Gray;
            CvtColor(Camera_Image,Gray);
            return Gray;
        }

            if (!m_cameraMatrix.empty() && !m_distCoeffs.empty()) {
                cv::Size imageSize;
                imageSize.width = Camera_Image.cols;
                imageSize.height = Camera_Image.rows;
                Size newImageSize = imageSize;
                initUndistortRectifyMap(m_cameraMatrix, m_distCoeffs, Mat(),
                    getOptimalNewCameraMatrix(m_cameraMatrix, m_distCoeffs, imageSize, 1, newImageSize, 0), newImageSize, CV_16SC2, Si::GetIns()->map1, Si::GetIns()->map2);
                Si::GetIns()->ToRemap = true;

                std::cout << "可以映射了" << std::endl;
            }

        static cv::Mat rview;
        if (Si::GetIns()->ToRemap)
        {
#ifdef NO_MOTION
             printf("矫正图片\n");
#endif
            remap(Camera_Image, rview, Si::GetIns()->map1, Si::GetIns()->map2, INTER_LINEAR);
        }
        else
        {
#ifdef NO_MOTION
             printf("没有矫正图片\n");
#endif
            rview = Camera_Image.clone();
        }
       // int len = rview.cols>rview.rows?rview.cols:rview.rows;
        cv::Mat Rot_Mat = cv::getRotationMatrix2D(cv::Point2f(rview.cols / 2.0, rview.rows / 2.0), cam_angle, 1.0);
        cv::warpAffine(rview, Camera_Snap, Rot_Mat, rview.size());
        static cv::Mat Gray;
        CvtColor(Camera_Snap, Gray, CV_BGR2GRAY);
        return Gray;
    }

}

#endif // CAMERAS_H

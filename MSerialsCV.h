#pragma once
#include <vector>
#include <string>
#include <list>
#include "common.h"
#include "opencv.hpp"

#ifndef __DLL__IMAGE__
#define __DLL__IMAGE__
//C++导出 vc14平台集
//#ifndef CVDLL_API
#define CVDLL_API __declspec(dllexport)
//#else
//#define CVDLL_API __declspec(dllimport)
//#endif

CVDLL_API std::string CvGetVersion();
CVDLL_API double CvSquareSize(cv::Mat mat, Control_Var & Ctrl_Var);																					//返回标定板对角线像素长度
CVDLL_API double CvCabli(double corner_size, Control_Var & Ctrl_Var, std::string & error_info, double ThresHold = 60, std::string dir = "CABLI");	//进行标定
CVDLL_API void CvCvtColor(const cv::Mat & org, cv::Mat & dst, int sel);																				//黑白彩色转换
CVDLL_API void CvLoadModelContours(cv::String FileName, Control_Var & Ctrl_Var);																	//功能 读取FileName的图片，将轮廓放在Ctrl var的modelcontours里面 并且图片存在Ctrl_Var Buffer里面
CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsHorizentalAI(cv::Mat &InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);				//智能横向
CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsHorizental(cv::Mat &InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);				//横向
CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsCircleVerticalAI(cv::Mat & InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);		//智能纵向
CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsCircleVertical(cv::Mat &InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);			//纵向
CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsDoubleHorizental(cv::Mat &InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);			//双面横向 不必要
CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsDoubleVertical(cv::Mat & InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);			//双面纵向 不必要

CVDLL_API bool CvInRange(cv::Mat & InputArray, std::vector<cv::Point> Image_Contours);						//确认图片的黑点是否在轮廓里面
CVDLL_API bool CvInRangeInvBias(cv::Mat & InputArray, const std::vector<cv::Point>& Image_Contours, const int x, const int y);
CVDLL_API bool CvInRangeInv(cv::Mat & InputArray, std::vector<cv::Point> Image_Contours);					//确认图片的白点是否在轮廓里面
//创建垂直镜像
CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursVertical(std::vector<std::vector<cv::Point>> model_contours);
CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursHorizental_Ex(std::vector<std::vector<cv::Point>> model_contours, cv::Mat * Out = nullptr);
//创建水平镜像
CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursHorizental(std::vector<std::vector<cv::Point>> model_contours);
//创建水平镜像
//找到轮廓所有的点去遍历
CVDLL_API void GetPointsContours(cv::Mat &InputArray, std::vector<cv::Point> &pts);

/*
CVDLL_API std::string CvGetVersion();

CVDLL_API double CvSquareSize(cv::Mat mat, Control_Var & Ctrl_Var);

CVDLL_API double CvCabli(double corner_size, Control_Var & Ctrl_Var, std::string & error_info, double ThresHold = 60, std::string dir = "CABLI");

CVDLL_API void CvCvtColor(const cv::Mat & org, cv::Mat & dst, int sel);
//功能 读取FileName的图片，将轮廓放在Ctrl var的modelcontours里面 并且图片存在Ctrl_Var Buffer里面
CVDLL_API void CvLoadModelContours(cv::String FileName, Control_Var & Ctrl_Var);

CVDLL_API void CvGetAdjustContours(const std::vector<std::vector<cv::Point>>& InputArray, std::vector<std::vector<cv::Point>>& OutputArray);

CVDLL_API void CvGetDialateContours(const std::vector<std::vector<cv::Point>>& InputArray, std::vector<cv::Point>& OutputContours, cv::Rect boundRect, int dilate_para);

CVDLL_API int CvFindMinMove(std::vector<std::vector<cv::Point>> model_contours);

CVDLL_API void CvDisplayMatImg(cv::Mat InputArray, void* pWnd_, bool Inv = false);

CVDLL_API void CvDispContours(cv::Mat & InputArray, std::vector<std::vector<cv::Point>> Contours, Control_Var & Ctrl_Var, cv::Scalar Color = cv::Scalar(245, 255, 10));

CVDLL_API std::list<std::vector<cv::Point>> CvGeAllPointsCircleVerticalAI(cv::Mat & InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);

CVDLL_API std::list<std::vector<cv::Point>> __stdcall CvGeAllPointsHorizentalAI(cv::Mat &InputArray, const std::vector<std::vector<cv::Point>> &model_contours, Control_Var& Ctrl_Var);

//确认图片的黑点是否在轮廓里面
CVDLL_API bool CvInRange(cv::Mat & InputArray, std::vector<cv::Point> Image_Contours);

CVDLL_API bool CvInRangeInvBias(cv::Mat & InputArray, const std::vector<cv::Point>& Image_Contours, const int x, const int y);
//确认图片的白点是否在轮廓里面
CVDLL_API bool CvInRangeInv(cv::Mat & InputArray, std::vector<cv::Point> Image_Contours);
//创建垂直镜像
CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursVertical(std::vector<std::vector<cv::Point>> model_contours);

CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursHorizental_Ex(std::vector<std::vector<cv::Point>> model_contours, cv::Mat * Out = nullptr);

//CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursHorizental_Ex(std::vector<std::vector<cv::Point>> model_contours, bool isAngle = false);
//创建水平镜像
CVDLL_API std::vector<std::vector<cv::Point>> CvGetDoubleContoursHorizental(std::vector<std::vector<cv::Point>> model_contours);
//创建水平镜像
//找到轮廓所有的点去遍历
CVDLL_API void GetPointsContours(cv::Mat &InputArray, std::vector<cv::Point> &pts);

CVDLL_API void CvPointsMoveMP(std::vector<cv::Point> InputArray, std::vector<cv::Point>& OutputArray, cv::Point pt);

CVDLL_API void CvPointsMove(std::vector<cv::Point> InputArray, std::vector<cv::Point>& OutputArray, cv::Point pt);
*/
#endif

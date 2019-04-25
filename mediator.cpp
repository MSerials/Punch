#include "mediator.h"

//unsigned int Mediator::InitCameraAndMotionCard(void* pLVOID);

void Mediator::IOC(){
    printf("ddd");
}


void Mediator::DigitalProcess()
{
    //副本
    //最初对位所使用的点
    cv::Point FirstPoint(0,0),ControlPoint(0,0);
    std::list<std::vector<cv::Point2d>> Points = DxfPoints;
    int Counter = 0;
    for(auto &pts:Points){
        for(auto &pt:pts){
            if( 0==pt.x) printf("left point\n");
            Counter++;
        }
    }
    if(Counter>0){
     cv::Point2d mm = Points.front().at(0);
     Excv::mm_to_pls(mm,FirstPoint);
    }

    long long lastCommand = MachineState;
    //每行1mm精度
    while(!Points.empty()){
        std::vector<cv::Point2d> pts = Points.front();
        for(auto &pt : pts){
            cv::Point WillPunched,WillMoved;
            Excv::mm_to_pls(pt,WillPunched);
            WillMoved.x = Ctrl_Var.CurrentXAxisPostion - (WillPunched.x - FirstPoint.x);
            WillMoved.y = Ctrl_Var.CurrentYAxisPostion + (WillPunched.y - FirstPoint.y);
            motion::GetIns()->CurrentCard()->absolute_move(X_AXIS_MOTOR, WillMoved.x, 50, X_AXIS_SPEED,2);
            motion::GetIns()->CurrentCard()->absolute_move(Y_AXIS_MOTOR, WillMoved.y, 50, Y_AXIS_SPEED,2);
            clock_t clk = clock();
            while (true) {
                if((clock()-clk) > AN_HOUR)
                {
                    //停止冲压 超时
                    MachineOp(MACHINE_STOP);
                    UpdateHistory("超时一个小时");
                    return;
                }

                if(PAUSE == (PAUSE&MachineState))
                {
                        lastCommand = PAUSE;
                        continue;
                }else if(STOP == (STOP&MachineState))
                {
                    lastCommand = STOP;
                    MachineOp(MACHINE_STOP);
                    UpdateHistory("按下了停止");
                    return;
                }else if(RUN == (RUN&MachineState))
                {
                    if(PAUSE == (PAUSE&lastCommand))
                    {
                        motion::GetIns()->CurrentCard()->absolute_move(X_AXIS_MOTOR, WillMoved.x, 50, X_AXIS_SPEED,2);
                        motion::GetIns()->CurrentCard()->absolute_move(Y_AXIS_MOTOR, WillMoved.y, 50, Y_AXIS_SPEED,2);
                    }
                    lastCommand = RUN;
                }
                else
                {
                    lastCommand = STOP;
                    MachineOp(MACHINE_STOP);
                    UpdateHistory("未知命令");
                    return;
                }


                int XDis = abs(static_cast<int>(motion::GetIns()->CurrentCard()->ReadInputBit(YANWEI_AXIS_CMD_POS,X_AXIS_MOTOR) - WillMoved.x));
                int YDis = abs(static_cast<int>(motion::GetIns()->CurrentCard()->ReadInputBit(YANWEI_AXIS_CMD_POS,Y_AXIS_MOTOR) - WillMoved.y));

                if(XDis <=  abs(POS_ACCURCY) && YDis <=  abs(POS_ACCURCY))
                {
                    break;
                }
                Sleep(5);
            }

            //等待冲压
            if(!CheckPunchTimeOut())           {
                UpdateHistory("六秒内没有发现冲床运动，件请检查通气通电状态和感应器是否安装正确。");
                return;
            }
            show_Counter();

        }
        Points.pop_front();
    }
    motion::GetIns()->CurrentCard()->absolute_move(X_AXIS_MOTOR, Ctrl_Var.CurrentXAxisPostion, 50, X_AXIS_SPEED,2);
    clock_t clk_of_wait_X = clock();
    while(true){
        Sleep(3);
        if(PAUSE == (PAUSE&MachineState))
        {
                lastCommand = PAUSE;
                continue;
        }else if(STOP == (STOP&MachineState))
        {
            lastCommand = STOP;
            MachineOp(MACHINE_STOP);
            UpdateHistory("按下了停止");
            return;
        }else if(RUN == (RUN&MachineState))
        {
            if(PAUSE == (PAUSE&lastCommand))
            {
                motion::GetIns()->CurrentCard()->absolute_move(X_AXIS_MOTOR, Ctrl_Var.CurrentXAxisPostion, 50, X_AXIS_SPEED,2);
                Sleep(5);
            }
            if(true == isAxisStop(X_AXIS_MOTOR) || (clock() - clk_of_wait_X) > 20000){
                break;
            }
            lastCommand = RUN;
        }
        else
        {
            lastCommand = STOP;
            MachineOp(MACHINE_STOP);
            UpdateHistory("未知命令");
            return;
        }
    }
    MachineOp(MACHINE_STOP);
    UpdateHistory("停止冲压");
}

void Mediator::Process()
{
    //参数准备
    try{
        Ctrl_Var.DealMethod = tagDealMethod::NORMAL;
    isSnapOver = false; evt_Punch.SetEvent(); evt_GetPoint.SetEvent();
    Sleep(25);
    if(!Excv::WaitValue(evt_Punch.State(),AN_HOUR) || !Excv::WaitValue(evt_GetPoint.State(),AN_HOUR)){
        UpdateMessage("超时了一个小时"); MachineOp(MACHINE_STOP);
    }
#ifdef NO_MOTION
  //  size_t Total = 0;
  //  for(auto v:Ctrl_Var.ModelsPostion) {Total += v.size();}
  //  printf(" 找到了%zd个点\n",Total);
#endif

    for (auto vpts3 : Ctrl_Var.Cal_ModelsPostion)
    {
        std::vector<cv::Point2l> vts;
        for (auto &pt : vpts3)
        {
            int x_ = pt.x* X_RATIO / X_DIS_PULS_RATIO;
            int y_ = pt.y* Y_RATIO / Y_DIS_PULS_RATIO;
            vts.push_back(cv::Point(x_, y_));
        }
        Ctrl_Var.ModelsPostion.push_back(vts);
    }
    //显示使用的，已经进行过偏移？
    DrawPts(Show_Image, Ctrl_Var, false);

    for (auto &vpts2 : Ctrl_Var.ModelsPostion) {
        for (auto &pt : vpts2)
        {
            //每个点进行偏移
            pt.y += Ctrl_Var.MovingForwardPuls;
        }
    }


    double rtHegiht = static_cast<double>(cv::boundingRect(Ctrl_Var.ModelContours.at(0)).height);
    double rtROI = CHECK_R2 - CHECK_R1;
    //最后一个点加上repeat高度就是末端，如果末端大于。。。针对小于2倍模具的
    if (rtROI < ROI_MODEL_RATIO * rtHegiht)
    {
        Ctrl_Var.Header_Pix_Ctrl = Ctrl_Var.Header_Pix;
        if(!Ctrl_Var.Cal_ModelsPostion.empty()){

         //   char msg[256] = {0};
         //   sprintf(msg,"找到的点排数为 %zd",Ctrl_Var.Cal_ModelsPostion.size());
         //   UpdateHistory(std::string(msg));

            Ctrl_Var.DealMethod = tagDealMethod::FINDTALEPIX;
            Ctrl_Var.Tale_Pix_Ctrl = Ctrl_Var.Cal_ModelsPostion.back().at(0).y;
        }

    }
    Ctrl_Var.DealMethod_Ctrl = Ctrl_Var.DealMethod;

#ifdef NO_MOTION
    cv::Point pix;
    cv::Point pls(Ctrl_Var.MovingForwardPuls,Ctrl_Var.MovingForwardPuls);

 //   pix.x = pls.x * X_DIS_PULS_RATIO / X_RATIO;
//    pix.y = pls.y * Y_DIS_PULS_RATIO / Y_RATIO;
    Excv::pls_to_pix(pls,pix);
    std::cout <<"前进了"<<Ctrl_Var.MovingForwardPuls<<"个脉冲"<< " 和 "<< pix.y<<"个像素"<<std::endl;
#endif

    HalconCpp::HObject Hobj;
    Excv::MatToHObj(Show_Image,Hobj);
    Excv::h_disp_obj(Hobj,MainWindowDispHd);
    static int runTime = 0;
    char Msg[256] = {0};
    sprintf(Msg,"循环次数%d",runTime++);
    UpdateMessage(std::string(Msg));
    }catch(cv::Exception ex)
    {
        MachineOp(MACHINE_STOP);
        UpdateMessage(std::string(ex.what()));
    }
    catch(MException e)
    {
        MachineOp(MACHINE_STOP);
        UpdateMessage(std::string(e.what()));
    }
    //准备统计和显示
}


bool Mediator::Get_Points(std::list<std::vector<cv::Point>> &Points)
{
   // UpdateMessage("开始拍照1");
    if(false == WaitAxisDone(Y_AXIS_MOTOR,10000,-2,0,MachineState))
    {
        UpdateMessage("Y轴运动超时,是否Y轴电机报警？"); MachineOp(MACHINE_STOP);
        return false;
    }
    try{
#ifndef NO_MOTION
    // UpdateMessage("开始拍照");
    //两张图片比较是否掉线,延时用于保证是Y轴完全停止的状态下拍照，否则造成模糊
    #define DELAY_SNAP 50
    clock_t clk = clock();
    cv::Mat Image_Try   = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED).clone();
    clock_t endClk = clock();
    if((endClk - clk) < DELAY_SNAP)
    {
        Sleep(DELAY_SNAP - (endClk - clk));
    }
    Image               = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED).clone();
    //图片掉线则重连
    if(true == MSerialsCamera::isEqual(Image,Image_Try))
    {
        for (int i = 0;;i++)
        {
            Sleep(100);
            if(MSerialsCamera::init_camera() > 0)
            {
                 Image       = MSerialsCamera::GetMachineImage(MSerialsCamera::IMAGE_FLIPED).clone();
                 break;
            }
            else if (i>30)
            {
                UpdateMessage("相机重连失败!，是否相机掉线？"); MachineOp(MACHINE_STOP);
                return false;
            }
        }
    }

#else
    Image = MSerialsCamera::Si::GetIns()->Image.clone();
#endif
    //通知拍摄完毕，说明可以动了
    isSnapOver = true;
    Ctrl_Var.Cal_ModelsPostion = Get_Points_Image(Image, Ctrl_Var.ModelContours, Ctrl_Var, LINE_METHOD);
    }catch(cv::Exception ex)
    {
        std::string info = "图像检测参数出错:" + std::string(ex.what());
        UpdateMessage(info); MachineOp(MACHINE_STOP);
        return false;
    }
    catch(std::out_of_range ex)
    {
        std::string info = "图像检测参数出错:" + std::string(ex.what());
        UpdateMessage(info); MachineOp(MACHINE_STOP);
        return false;
    }
    return true;
}


//控制过程选择，是视觉还是数控
unsigned int __stdcall Mediator::Process_(void* pLVOID){pLVOID  = nullptr;
   static clock_t maxTime = 0, coutt = 0;
    for(;;){
        ::WaitForSingleObject(Mediator::GetIns()->evt_Process.get(),INFINITE);
#ifdef NO_MOTION
        clock_t clk = clock();
        Mediator::GetIns()->Process();
        clock_t t = clock()-clk;
        coutt++;
        if((coutt)>80)
        {
            coutt = 0;
            maxTime = 0;
        }
        if(t > maxTime)
        {
            maxTime = t;
        }
        printf("循环时间%ld 最大循环时间 %ld %d\n",clock()-clk,maxTime,Mediator::GetIns()->evt_Process.State());
#else
        if(PUNCHMODE == CAMERAMODE)
        {
            Mediator::GetIns()->Process();
        }
        else
        {
            Mediator::GetIns()->DigitalProcess();
        }
#endif
    }
}

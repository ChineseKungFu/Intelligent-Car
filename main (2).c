                              //开环控制智能车
//头文件
#include "common.h"
#include "include.h"
#include "math.h"
                                          



/*==================================取中线部分==================================*/
int ROW,Left_COL=40,Right_COL=40;
int Midline_ROW[60]={0};
int Midline_ROW_Change=0;
int Midline_Count=0;
int Left_Black_COL;
int Right_Black_COL;
int Left_Black_Count=0;
int Right_Black_Count=0;
int Right_White;
int Left_White;
int Left_Black_COL_OLD;
int Right_Black_COL_OLD;
int ROW=59;
int COL=79;
int start_line_flag;
int car_stop=0;
int servo_control(void);
int stop;
int carstop=0;
int shiziflag;
int wandao[60]={40,40,40,40,40,39,39,38,38,37,37,36,36,35,35,34,34,33,33,32,32,31,31,30,30,29,29,28,28,27,26,26,25,25,24,24,23,23,22,22,21,21,20,20,19,19,18,18,17,17,16,16,15,15};
//int yuanhuan[60]={40,40,40,40,40,32,22,24,24,24,26,26,26,28,28,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,29,29,29,28,28,28,27,27,26,26,25,25,24,24,24,23,22,19,16,13,10,7,4,1};
/*==================================PID部分==================================*/
float error=0.0;
float last_error=0.0;
float kp;
float kd;
//int last_last_error=0;
//int last_PWMDITY01=287;
float Servo_PWM=290.0;


/*==================================控制部分==================================*/
//电机设定
#define MOTOR1_IO   PTD15
#define MOTOR2_IO   PTA19
#define MOTOR3_IO   PTA4
#define MOTOR4_IO   PTA24


#define MOTOR1_PWM_IO  FTM0_CH0
#define MOTOR2_PWM_IO  FTM0_CH6
#define MOTOR3_PWM_IO  FTM0_CH2
#define MOTOR4_PWM_IO  FTM0_CH3

#define MOTOR_HZ    20*1000//(20*1000)//滑行模式下，频率应该是 30~100.常规模式下，频率应该是 20k 左右
#define MOTOR_DUTY  80


//摄像头设定
uint8 imgbuff[CAMERA_SIZE];         //定义存储接收图像的数组
uint8 img[CAMERA_H][CAMERA_W];     //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
                                  //假如需要二维数组，只需要改成 uint8 img[CAMERA_H][CAMERA_W];
                                 //imgbuff是采集的缓冲区，img是解压后的缓冲区。imgbuff用于采集图像，img用于图像处理.

//函数声明(中断服务)
void PORTA_IRQHandler();
void DMA0_IRQHandler();
//发送上位机的函数声明
void sendimg(void *imgaddr, uint32 imgsize);




//舵机设定
#define S3010_FTM   FTM2
#define S3010_CH    FTM_CH0
#define S3010_HZ    (125)





  void main()
{
    /*===============================初始化部分===============================*/
    //电机初始化
    #if 1
    ftm_pwm_init(FTM0, FTM_CH0,MOTOR_HZ,100);      //初始化 电机 PWM     
    
    ftm_pwm_init(FTM0, FTM_CH6,MOTOR_HZ,100);      //初始化 电机 PWM
    
    ftm_pwm_init(FTM0, FTM_CH2,MOTOR_HZ,100);      //初始化 电机 PWM
    
    ftm_pwm_init(FTM0, FTM_CH3,MOTOR_HZ,100);      //初始化 电机 PWM
    //IO管脚配置（电机）
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
    #endif    
       
    //摄像头初始化
    camera_init(imgbuff);//这里设定  imgbuff 为采集缓冲区！！！！！！
     //配置中断服务函数
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置DMA0的中断服务函数为 DMA0_IRQHandler   
    //舵机初始化
    ftm_pwm_init(FTM2, FTM_CH0,125,290); //初始化 舵机 PWM（290为中值）
    
    
    
    /*===============================控制部分===============================*/
while(1)
{     
      
      camera_get_img();                                //摄像头获取图像                 
     //vcan_sendimg(imgbuff, sizeof(imgbuff));        
     img_extract(img, imgbuff, CAMERA_SIZE);//解压为灰度图像       
     
     start_line_flag=0;
     
     
     Left_White=0;
     Right_White=0;
     Left_Black_COL_OLD=0;
     Right_Black_COL_OLD=0;
     ROW=50;
     //Left_COL=40;
     //Right_COL=40;
     for(;ROW>=5;ROW--) 
     {                         
            //从坐标零点开始往左搜索找黑点
            for(;Left_COL>0;Left_COL--)
            {
               if(img[ROW][Left_COL]==0xFF&&img[ROW][Left_COL-1]==0x00)//找到黑点
                   { 
                      Left_Black_COL=Left_COL;
                      Left_Black_COL_OLD=Left_Black_COL;
                      break;
                    }          
              
            };
         
         //从坐标零点开始往右搜索找黑点
           for(;Right_COL<=79;Right_COL++)
            {
               if(img[ROW][Right_COL]==0x00&&img[ROW][Right_COL-1]==0xFF)//找到黑点
               {
               Right_Black_COL=Right_COL;
               Right_Black_COL_OLD=Right_Black_COL;
               break;
               }                                           
            };
        
        //提取中线
        if(Right_Black_COL_OLD!=0&&Left_Black_COL_OLD!=0)//直线检测
              {
                Midline_ROW[ROW]=(Left_Black_COL_OLD+Right_Black_COL_OLD)/2;
                if(ROW>=25&&ROW<=50)
                 {
                   if(abs(Right_Black_COL_OLD-Left_Black_COL_OLD)<=45&&abs(Right_Black_COL_OLD-Left_Black_COL_OLD)>=15)//障碍检测
                   {
                    if((Left_Black_COL_OLD-20)>0)//左侧
                    {
                    Midline_ROW[ROW]=60;
                    }
                    else if((Right_Black_COL_OLD-65)<0) //右侧
                    {
                    Midline_ROW[ROW]=10;
                    }; 
                   };                              
                 };
              }
        else if(Right_Black_COL_OLD!=0&&Left_Black_COL_OLD==0)//左弯道检测
              {
               
                Midline_ROW[ROW]=((Right_Black_COL_OLD-wandao[ROW])/2);
                if(Midline_ROW[ROW]<0)
                Midline_ROW[ROW]=0;
              }
        else if(Right_Black_COL_OLD==0&&Left_Black_COL_OLD!=0)//右弯道检测
              {
                Midline_ROW[ROW]=((80+Left_Black_COL_OLD+wandao[ROW])/2);
                if(Midline_ROW[ROW]>79)
                Midline_ROW[ROW]=79;
              }
        else if(Right_Black_COL_OLD==0&&Left_Black_COL_OLD==0)//十字和圆环检测
              {         
                 //Midline_ROW[ROW]=(Right_Black_COL+Left_Black_COL)/2;
                   Midline_ROW[ROW]=Midline_ROW[ROW+1];
                 //shiziflag++;    
              };
        
        
        Left_COL=Midline_ROW[ROW];
        Right_COL=Midline_ROW[ROW];
        img[ROW][Midline_ROW[ROW]]=0x00;
                   
        };
          
    
        ftm_pwm_duty(FTM0, FTM_CH2,30);
        ftm_pwm_duty(FTM0, FTM_CH3,0);
        ftm_pwm_duty(FTM0, FTM_CH6,0);
        ftm_pwm_duty(FTM0, FTM_CH0,30);//电机*/
        
        ftm_pwm_duty(FTM2, FTM_CH0,servo_control()); 
        
      for(int i=45;i>40;i--)
      {
       for(int j=60;j>20;j--)
        {
         if(img[i][j-1]==0xFF&&img[i][j]==0xFF&&img[i][j+1]==0x00&&img[i][j+2]==0x00)
         {
         start_line_flag++;
         }
        }
       };
      
      
      
      
      if(start_line_flag>10)//10 15 20 25
      {
       ftm_pwm_duty(FTM0, FTM_CH2,0);
       ftm_pwm_duty(FTM0, FTM_CH3,8);
       ftm_pwm_duty(FTM0, FTM_CH6,8);
       ftm_pwm_duty(FTM0, FTM_CH0,0);
       ftm_pwm_duty(FTM2, FTM_CH0,290);
       DELAY_MS(5000);
      };
      //vcan_sendimg(img, sizeof(img));                  //发送到上位机
     };
     
        

    



}//while      
       
//main
    







/*===============================声明函数主体部分===============================*/
//摄像头中断服务部分

void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}


void DMA0_IRQHandler()//DMA0的中断服务
{
    camera_dma();
}

//发送图像到串口助手
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_摄像头串口调试 使用的命令

    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //先发送命令

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
}


//PID
int servo_control(void)

{

   int i;

   int SteerSum=0;
   kp=0.7;
   kd=5.0;
   
   for(i=45;i>25;i--)  //仅对近处的行取平均值
    
     
    SteerSum+=(Midline_ROW[i]-40);//注意：Fit_Middleline[i]-Img_Col/2时对应于//Servo_PWM + servo_pwm_middle时摄像头正向安装可以正常跑，，，

    error=(int)(SteerSum/20);
    
    
    Servo_PWM=(kp*error)+kd*(error-last_error);

    Servo_PWM= 290-Servo_PWM;
  
    Servo_PWM=(int)(Servo_PWM);

    last_error=error;
  
  

  if(Servo_PWM > 308)  //限定舵机打角范围，防止超过两个轮子所能转过的范//围，servo_pwm_max是轮子最大的转角，可调节占空比使舵机打角后轮子所能转的角度对//应的最大占空比，这样便可得出轮子所能转过的最大占空比和最小占空比。

    Servo_PWM = 308;

  

  if(Servo_PWM < 272)

    Servo_PWM =272;

  

   if((error<3)&& (error>-3))    //偏差太小就不改变舵机角度

     Servo_PWM=290;    //使用原来舵机的值 

   

   

  return (int)(Servo_PWM);

 

}



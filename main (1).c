                              //intelligent car
//header document

#include "common.h"
#include "include.h"
#include "math.h"



/*==================================set variables==================================*/
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
/*==================================PID which control servo==================================*/
float error=0.0;
float last_error=0.0;
float kp;
float kd;
//int last_last_error=0;
//int last_PWMDITY01=287;
float Servo_PWM=290.0;


/*==================================Control==================================*/
//motor set

#define MOTOR1_IO   PTD15
#define MOTOR2_IO   PTA19
#define MOTOR3_IO   PTA4
#define MOTOR4_IO   PTA24


#define MOTOR1_PWM_IO  FTM0_CH0
#define MOTOR2_PWM_IO  FTM0_CH6
#define MOTOR3_PWM_IO  FTM0_CH2
#define MOTOR4_PWM_IO  FTM0_CH3

#define MOTOR_HZ    20*1000//(20*1000)
#define MOTOR_DUTY  80


//camera set
uint8 imgbuff[CAMERA_SIZE];
uint8 img[CAMERA_H][CAMERA_W];



//interrupt
void PORTA_IRQHandler();
void DMA0_IRQHandler();
//sent the image to PC
void sendimg(void *imgaddr, uint32 imgsize);




//servo set
#define S3010_FTM   FTM2
#define S3010_CH    FTM_CH0
#define S3010_HZ    (125)





  void main()
{
    /*===============================initialization===============================*/
    //motor initialized
    #if 1
    ftm_pwm_init(FTM0, FTM_CH0,MOTOR_HZ,100);

    ftm_pwm_init(FTM0, FTM_CH6,MOTOR_HZ,100);

    ftm_pwm_init(FTM0, FTM_CH2,MOTOR_HZ,100);

    ftm_pwm_init(FTM0, FTM_CH3,MOTOR_HZ,100);
    //GPIO
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
    #endif

    //camera initialized
    camera_init(imgbuff);
     //interrupt set
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);
    //servo initialized
    ftm_pwm_init(FTM2, FTM_CH0,125,290); //290 is middle angle of servo



    /*===============================MAIN===============================*/
while(1)
{

      camera_get_img();
     //vcan_sendimg(imgbuff, sizeof(imgbuff));
     img_extract(img, imgbuff, CAMERA_SIZE);

     start_line_flag=0;


     Left_White=0;
     Right_White=0;
     Left_Black_COL_OLD=0;
     Right_Black_COL_OLD=0;
     ROW=50;
     //Left_COL=40;
     //Right_COL=40;
     for(;ROW>=5;ROW--)//search
     {
            //search to the left
            for(;Left_COL>0;Left_COL--)
            {
               if(img[ROW][Left_COL]==0xFF&&img[ROW][Left_COL-1]==0x00)//找到黑点
                   {
                      Left_Black_COL=Left_COL;
                      Left_Black_COL_OLD=Left_Black_COL;
                      break;
                    }

            };

         //search to the right
           for(;Right_COL<=79;Right_COL++)
            {
               if(img[ROW][Right_COL]==0x00&&img[ROW][Right_COL-1]==0xFF)//找到黑点
               {
               Right_Black_COL=Right_COL;
               Right_Black_COL_OLD=Right_Black_COL;
               break;
               }
            };

        //midline get
        if(Right_Black_COL_OLD!=0&&Left_Black_COL_OLD!=0)//straight
              {
               Midline_ROW[ROW]=(Left_Black_COL_OLD+Right_Black_COL_OLD)/2;
              }
        else if(Right_Black_COL_OLD!=0&&Left_Black_COL_OLD==0)//left
              {

                Midline_ROW[ROW]=((Right_Black_COL_OLD-wandao[ROW])/2);
                if(Midline_ROW[ROW]<0)
                Midline_ROW[ROW]=0;
              }
        else if(Right_Black_COL_OLD==0&&Left_Black_COL_OLD!=0)//right
              {
                Midline_ROW[ROW]=((80+Left_Black_COL_OLD+wandao[ROW])/2);
                if(Midline_ROW[ROW]>79)
                Midline_ROW[ROW]=79;
              }
        else if(Right_Black_COL_OLD==0&&Left_Black_COL_OLD==0)//cross
              {
                 Midline_ROW[ROW]=(Right_Black_COL+Left_Black_COL)/2;
                 //shiziflag++;
              };
        Left_COL=Midline_ROW[ROW];
        Right_COL=Midline_ROW[ROW];
        img[ROW][Midline_ROW[ROW]]=0x00;

        };


        ftm_pwm_duty(FTM0, FTM_CH2,25);
        ftm_pwm_duty(FTM0, FTM_CH3,0);
        ftm_pwm_duty(FTM0, FTM_CH6,0);
        ftm_pwm_duty(FTM0, FTM_CH0,25);//motor*/

        ftm_pwm_duty(FTM2, FTM_CH0,servo_control());

      for(int i=30;i>25;i--)
      {
       for(int j=60;j>20;j--)
        {
         if(img[i][j-1]==0xFF&&img[i][j]==0xFF&&img[i][j+1]==0x00&&img[i][j+2]==0x00)
         {
         start_line_flag++;
         }
        }
      };


      if(start_line_flag>10)
      {
       ftm_pwm_duty(FTM0, FTM_CH2,0);
       ftm_pwm_duty(FTM0, FTM_CH3,5);
       ftm_pwm_duty(FTM0, FTM_CH6,5);
       ftm_pwm_duty(FTM0, FTM_CH0,0);
       ftm_pwm_duty(FTM2, FTM_CH0,290);
       DELAY_MS(5000);
      };
        //vcan_sendimg(img, sizeof(img));                  //sent to PC
     };







}//while

//main








/*===============================Function===============================*/
//camera function with interrupt

void PORTA_IRQHandler()
{
    uint8  n = 0;
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;

    n = 29;
    if(flag & (1 << n))
    {
        camera_vsync();
    }
#if 0
    n = 28;
    if(flag & (1 << n))
    {
        camera_href();
    }
#endif
}


void DMA0_IRQHandler()
{
    camera_dma();
}

//sent image to PC
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //debug order
    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //sent order

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //sent image
}


//PID
int servo_control(void)

{

   int i;

   int SteerSum=0;
   kp=0.65;
   kd=7.5;//7.3

   for(i=45;i>25;i--)  //get average

    SteerSum+=(Midline_ROW[i]-40);

    error=(int)(SteerSum/20);

    if(fabs(error)<3.0)
      kp=0.1;
    else if(fabs(error)>3.0&&fabs(error)<5.0)
      kp=0.3;
    else if(fabs(error)>5.0&&fabs(error)<10.0)
      kp=0.5;
    else if(fabs(error)>10.0)
      kp=0.9;
    Servo_PWM=(kp*error)+kd*(error-last_error);

    Servo_PWM= 290-Servo_PWM;

    Servo_PWM=(int)(Servo_PWM);

    last_error=error;



  if(Servo_PWM > 308)  //limit angle of servo

    Servo_PWM = 308;



  if(Servo_PWM < 272)

    Servo_PWM =272;



   if((error<3)&& (error>-3))

     Servo_PWM=290;




  return (int)(Servo_PWM);



}


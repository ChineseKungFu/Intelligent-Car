                              //�����������ܳ�
//ͷ�ļ�
#include "common.h"
#include "include.h"
#include "math.h"
                                          



/*==================================ȡ���߲���==================================*/
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
/*==================================PID����==================================*/
float error=0.0;
float last_error=0.0;
float kp;
float kd;
//int last_last_error=0;
//int last_PWMDITY01=287;
float Servo_PWM=290.0;


/*==================================���Ʋ���==================================*/
//����趨
#define MOTOR1_IO   PTD15
#define MOTOR2_IO   PTA19
#define MOTOR3_IO   PTA4
#define MOTOR4_IO   PTA24


#define MOTOR1_PWM_IO  FTM0_CH0
#define MOTOR2_PWM_IO  FTM0_CH6
#define MOTOR3_PWM_IO  FTM0_CH2
#define MOTOR4_PWM_IO  FTM0_CH3

#define MOTOR_HZ    20*1000//(20*1000)//����ģʽ�£�Ƶ��Ӧ���� 30~100.����ģʽ�£�Ƶ��Ӧ���� 20k ����
#define MOTOR_DUTY  80


//����ͷ�趨
uint8 imgbuff[CAMERA_SIZE];         //����洢����ͼ�������
uint8 img[CAMERA_H][CAMERA_W];     //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
                                  //������Ҫ��ά���飬ֻ��Ҫ�ĳ� uint8 img[CAMERA_H][CAMERA_W];
                                 //imgbuff�ǲɼ��Ļ�������img�ǽ�ѹ��Ļ�������imgbuff���ڲɼ�ͼ��img����ͼ����.

//��������(�жϷ���)
void PORTA_IRQHandler();
void DMA0_IRQHandler();
//������λ���ĺ�������
void sendimg(void *imgaddr, uint32 imgsize);




//����趨
#define S3010_FTM   FTM2
#define S3010_CH    FTM_CH0
#define S3010_HZ    (125)





  void main()
{
    /*===============================��ʼ������===============================*/
    //�����ʼ��
    #if 1
    ftm_pwm_init(FTM0, FTM_CH0,MOTOR_HZ,100);      //��ʼ�� ��� PWM     
    
    ftm_pwm_init(FTM0, FTM_CH6,MOTOR_HZ,100);      //��ʼ�� ��� PWM
    
    ftm_pwm_init(FTM0, FTM_CH2,MOTOR_HZ,100);      //��ʼ�� ��� PWM
    
    ftm_pwm_init(FTM0, FTM_CH3,MOTOR_HZ,100);      //��ʼ�� ��� PWM
    //IO�ܽ����ã������
    gpio_init(MOTOR1_IO,GPO,LOW);
    gpio_init(MOTOR2_IO,GPO,LOW);
    gpio_init(MOTOR3_IO,GPO,LOW);
    gpio_init(MOTOR4_IO,GPO,LOW);
    #endif    
       
    //����ͷ��ʼ��
    camera_init(imgbuff);//�����趨  imgbuff Ϊ�ɼ�������������������
     //�����жϷ�����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����PORTA���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����DMA0���жϷ�����Ϊ DMA0_IRQHandler   
    //�����ʼ��
    ftm_pwm_init(FTM2, FTM_CH0,125,290); //��ʼ�� ��� PWM��290Ϊ��ֵ��
    
    
    
    /*===============================���Ʋ���===============================*/
while(1)
{     
      
      camera_get_img();                                //����ͷ��ȡͼ��                 
     //vcan_sendimg(imgbuff, sizeof(imgbuff));        
     img_extract(img, imgbuff, CAMERA_SIZE);//��ѹΪ�Ҷ�ͼ��       
     
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
            //��������㿪ʼ���������Һڵ�
            for(;Left_COL>0;Left_COL--)
            {
               if(img[ROW][Left_COL]==0xFF&&img[ROW][Left_COL-1]==0x00)//�ҵ��ڵ�
                   { 
                      Left_Black_COL=Left_COL;
                      Left_Black_COL_OLD=Left_Black_COL;
                      break;
                    }          
              
            };
         
         //��������㿪ʼ���������Һڵ�
           for(;Right_COL<=79;Right_COL++)
            {
               if(img[ROW][Right_COL]==0x00&&img[ROW][Right_COL-1]==0xFF)//�ҵ��ڵ�
               {
               Right_Black_COL=Right_COL;
               Right_Black_COL_OLD=Right_Black_COL;
               break;
               }                                           
            };
        
        //��ȡ����
        if(Right_Black_COL_OLD!=0&&Left_Black_COL_OLD!=0)//ֱ�߼��
              {
                Midline_ROW[ROW]=(Left_Black_COL_OLD+Right_Black_COL_OLD)/2;
                if(ROW>=25&&ROW<=50)
                 {
                   if(abs(Right_Black_COL_OLD-Left_Black_COL_OLD)<=45&&abs(Right_Black_COL_OLD-Left_Black_COL_OLD)>=15)//�ϰ����
                   {
                    if((Left_Black_COL_OLD-20)>0)//���
                    {
                    Midline_ROW[ROW]=60;
                    }
                    else if((Right_Black_COL_OLD-65)<0) //�Ҳ�
                    {
                    Midline_ROW[ROW]=10;
                    }; 
                   };                              
                 };
              }
        else if(Right_Black_COL_OLD!=0&&Left_Black_COL_OLD==0)//��������
              {
               
                Midline_ROW[ROW]=((Right_Black_COL_OLD-wandao[ROW])/2);
                if(Midline_ROW[ROW]<0)
                Midline_ROW[ROW]=0;
              }
        else if(Right_Black_COL_OLD==0&&Left_Black_COL_OLD!=0)//��������
              {
                Midline_ROW[ROW]=((80+Left_Black_COL_OLD+wandao[ROW])/2);
                if(Midline_ROW[ROW]>79)
                Midline_ROW[ROW]=79;
              }
        else if(Right_Black_COL_OLD==0&&Left_Black_COL_OLD==0)//ʮ�ֺ�Բ�����
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
        ftm_pwm_duty(FTM0, FTM_CH0,30);//���*/
        
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
      //vcan_sendimg(img, sizeof(img));                  //���͵���λ��
     };
     
        

    



}//while      
       
//main
    







/*===============================�����������岿��===============================*/
//����ͷ�жϷ��񲿷�

void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}


void DMA0_IRQHandler()//DMA0���жϷ���
{
    camera_dma();
}

//����ͼ�񵽴�������
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_����ͷ���ڵ��� ʹ�õ�����

    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //�ȷ�������

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��
}


//PID
int servo_control(void)

{

   int i;

   int SteerSum=0;
   kp=0.7;
   kd=5.0;
   
   for(i=45;i>25;i--)  //���Խ�������ȡƽ��ֵ
    
     
    SteerSum+=(Midline_ROW[i]-40);//ע�⣺Fit_Middleline[i]-Img_Col/2ʱ��Ӧ��//Servo_PWM + servo_pwm_middleʱ����ͷ����װ���������ܣ�����

    error=(int)(SteerSum/20);
    
    
    Servo_PWM=(kp*error)+kd*(error-last_error);

    Servo_PWM= 290-Servo_PWM;
  
    Servo_PWM=(int)(Servo_PWM);

    last_error=error;
  
  

  if(Servo_PWM > 308)  //�޶������Ƿ�Χ����ֹ����������������ת���ķ�//Χ��servo_pwm_max����������ת�ǣ��ɵ���ռ�ձ�ʹ�����Ǻ���������ת�ĽǶȶ�//Ӧ�����ռ�ձȣ�������ɵó���������ת�������ռ�ձȺ���Сռ�ձȡ�

    Servo_PWM = 308;

  

  if(Servo_PWM < 272)

    Servo_PWM =272;

  

   if((error<3)&& (error>-3))    //ƫ��̫С�Ͳ��ı����Ƕ�

     Servo_PWM=290;    //ʹ��ԭ�������ֵ 

   

   

  return (int)(Servo_PWM);

 

}



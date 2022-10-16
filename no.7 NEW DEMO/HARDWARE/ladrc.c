#include "LADRC.h"
LADRC_NUM ADRC_M3508_CHASIS[3];
LADRC_NUM ADRC_M3508_UP;
LADRC_NUM ADRC_M3508_YAW;
LADRC_NUM ADRC_M3508_TRANSATE;
/**
   *@Brief default ������
   *@Brief ���ݾ��� ts��0.2~0.3֮�� Wo��Wcѡ�����С��������b0
   *@Date@WangShun 2022-05-28 2022-07-03����
---------------------------------------------------
		      LADRC default������									
---------------------------------------------------
---------------------------------------------------
	ts	|	h	|	r	|   wc   |   w0  |	b0
---------------------------------------------------
	0.1	|	h	|	r	|  100   |  400  |	b0
---------------------------------------------------
   0.157|	h	|	r	|   64   |  224~255  |	b0
---------------------------------------------------
   0.158|	h	|	r	|   63   |  253  |	b0
---------------------------------------------------
   0.159|	h	|	r	|   63   |  252  |	b0
---------------------------------------------------
	0.16|	h	|	r	|   63   |  250  |	b0
---------------------------------------------------
	0.17|	h	|	r	|   59   |  235  |	b0
---------------------------------------------------
	0.18|	h	|	r	|   56   |  222  |	b0
---------------------------------------------------
	0.2	|	h	|	r	|   50   |  200  |	b0
---------------------------------------------------
	0.21|	h	|	r	|   48   |  190  |	b0
---------------------------------------------------
	0.22|	h	|	r	|   45   |  182  |	b0
---------------------------------------------------
	0.23|	h	|	r	|   43   |  174  |	b0
---------------------------------------------------
	0.24|	h	|	r	|   42   |  167  |	b0
---------------------------------------------------
	0.25|	h	|	r	|   40   |  160  |	b0
---------------------------------------------------
	0.26|	h	|	r	|   38   |  154  |	b0
---------------------------------------------------
	0.27|	h	|	r	|   37   |  148 |	b0
---------------------------------------------------
	0.28|	h	|	r	|   36   |  144  |	b0
---------------------------------------------------
	0.29|	h	|	r	|   34   |  138  |	b0
---------------------------------------------------
	0.3	|	h	|	r	|   33   |  133  |	b0
---------------------------------------------------
	0.4	|	h	|	r	|   25   |  100  |	b0
---------------------------------------------------
	0.5	|	h	|	r	|   20   |   80  |	b0
---------------------------------------------------
---------------------------------------------------
*/
 
 /**
  * ����˵�� LADRC��ʼ�ο�ֵ
  * 		WangShun��2022-07-03����
  */	
const float LADRC_Unit[5][5]=
{
	{0.005,20,100,400,0.5},
	{0.001,20,33,133,8},
	{0.005,100,20,80,0.5},
	{0.005,100,14,57,0.5},
	{0.005,100,50,10,1}
};
/**
  * ����˵����LADRC��ʼ��
  * 		WangShun��2022-07-03����
  */	
void LADRC_Init(LADRC_NUM *LADRC_TYPE1,float h,float r,float wc,float w0,float b0,float outputmax, float deadzone)
{
	LADRC_TYPE1->h=h; //��ʱʱ�估ʱ�䲽��
  LADRC_TYPE1->r=r; //�����ٶȲ���
  LADRC_TYPE1->wc=wc; //�۲�������
  LADRC_TYPE1->w0=w0; //״̬�����ʴ���
	LADRC_TYPE1->b0=b0; //ϵͳ����
	LADRC_TYPE1->outputmax=outputmax;
	LADRC_TYPE1->deadzone=deadzone;
}
/**
  * ����˵����LADRCȱʡ
  * 		WangShun��2022-07-03����
  */
void LADRC_REST(LADRC_NUM *LADRC_TYPE1)
{
	LADRC_TYPE1->z1= 0; //��ʱʱ�估ʱ�䲽��
  LADRC_TYPE1->z2 =0; //�����ٶȲ���
  LADRC_TYPE1->z3=0; //�۲�������
	LADRC_TYPE1->DError = LADRC_TYPE1->Error = LADRC_TYPE1->SumError = LADRC_TYPE1->output = LADRC_TYPE1->LastError = LADRC_TYPE1->PrevError = LADRC_TYPE1->errormax = 0.0f; 
	LADRC_TYPE1->first_flag = 1;

}
/**
  * ��������void ADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect)
  * ����˵����LADRC����΢�ֲ���
  * @param[in]	��ڲ���������ֵExpect(v0)���ֵv1,v2
  * @par �޸���־
  * 		WangShun��2022-05-28����
  */
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect)
{
    float fh= -LADRC_TYPE1->r*LADRC_TYPE1->r*(LADRC_TYPE1->v1-Expect)-2*LADRC_TYPE1->r*LADRC_TYPE1->v2;
    LADRC_TYPE1->v1+=LADRC_TYPE1->v2*LADRC_TYPE1->h;
    LADRC_TYPE1->v2+=fh*LADRC_TYPE1->h;
}
/**
  * ��������LADRC_ESO(LADRC_NUM *LADRC_TYPE1,float FeedBack)
  * ����˵����LADRC����״̬�۲���
  * @param[in]
  * @par �޸���־
  * 		WangShun��2022-07-03����
  */
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,float FeedBack)
{
    float Beita_01=3*LADRC_TYPE1->w0;
    float Beita_02=3*LADRC_TYPE1->w0*LADRC_TYPE1->w0;
    float Beita_03=LADRC_TYPE1->w0*LADRC_TYPE1->w0*LADRC_TYPE1->w0;

    float e= LADRC_TYPE1->z1-FeedBack;
    LADRC_TYPE1->z1+= (LADRC_TYPE1->z2 - Beita_01*e)*LADRC_TYPE1->h;
    LADRC_TYPE1->z2+= (LADRC_TYPE1->z3 - Beita_02*e + LADRC_TYPE1->b0*LADRC_TYPE1->u)*LADRC_TYPE1->h;
    LADRC_TYPE1->z3+=-Beita_03*e*LADRC_TYPE1->h;
}
/**
   *@Brief  LADRC_LSEF
   *@Date   ���Կ�����
			WangShun��2022-07-03����
   */
void LADRC_LF(LADRC_NUM *LADRC_TYPE1)
{
    float Kp=LADRC_TYPE1->wc*LADRC_TYPE1->wc;
    float Kd=2*LADRC_TYPE1->wc;
	/**
       *@Brief  ���Կ�����������kd = 2wc
       *@Before Kd=3*LADRC_TYPE1->wc;
       *@Now    Kd=2*LADRC_TYPE1->wc;
       *@WangShun  2022-04-27  ע��
       */
    float e1=LADRC_TYPE1->v1-LADRC_TYPE1->z1;
    float e2=LADRC_TYPE1->v2-LADRC_TYPE1->z2;
    float u0=Kp*e1+Kd*e2;
    LADRC_TYPE1->u=(u0-LADRC_TYPE1->z3)/LADRC_TYPE1->b0;
	if(LADRC_TYPE1->u>LADRC_TYPE1->outputmax)//�޷��ǵø���ʵ���޸�
		LADRC_TYPE1->u=LADRC_TYPE1->outputmax;
	else if(LADRC_TYPE1->u<-LADRC_TYPE1->outputmax)
		LADRC_TYPE1->u=-LADRC_TYPE1->outputmax;
}
/**
  * LADRC���ƺ��� .
  * ������������ѭ���м���
  * @par ����
  * @par �޸���־
  * @WangShun  2022-07-03  ע��
  */
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,float RealTimeOut,float Expect)
{
	float  Expect_Value = Expect;
	float  Measure = RealTimeOut;
    LADRC_TD(LADRC_TYPE1,Expect_Value);
    LADRC_ESO(LADRC_TYPE1,Measure); 
    LADRC_LF(LADRC_TYPE1);
}


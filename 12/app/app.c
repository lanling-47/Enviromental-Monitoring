#include <string.h>
#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "lorawan_node_driver.h"
#include "hdc1000.h"
#include "sensors_test.h"
#include "ST7789v.h"
#include "XPT2046.h"
#include "opt3001.h"
#include "MPL3115.h"
#include "mma8451.h"
#include "flash_.h"
extern Pen_Holder Pen_Point;
extern DEVICE_MODE_T device_mode;
extern DEVICE_MODE_T *Device_Mode_str;
down_list_t *pphead = NULL;

u16 posx=0,posy=0;//��ʱ����������
u8 ifpoint=0;//�Ƿ��е㰴�£�������
unsigned char cnt[]={"00000 00000"};//��ʾ������꣬������

//ȫ�����޸Ĺ����µط�
//����˶�ʱ��TIM4����ؿ�(tim.c/tim.h)
//����˶�ʱ��TIM4���жϺ���(stm32l4xx_it.c)
//�޸��˳�ʼ����������غ���(lorawan_node_driver.c)
//�޸��˰���������(key.c)
//�޸��˴�������ʼ������(ST7789v.c)
//�޸������߼�������(���ļ�)
//���ߣ��𶡺�
//������ʱ�䣺2021/6/12/23/06
//���±���Ϊ�������
u8 settingmod=0;//�Ƿ�������ģʽ��1��ʾ��
u8 sensor_type=0;//���������ͣ�0123����4�ִ�����
u8 upload_span=1;//�ϴ����/s 1 2 3 4 5
u8 upload_times=5;//ƽ������/�� 5 10 15 20 25 30
u8 temp_chr_2[]={"00"};//��ʱ���飬������Ļ��ʾ�ϴ������ƽ������
u8 upload_data[9]={0xAA,0xF5,0x6B,0x01,0x00,0x00,0x00,0x00,0x0f};//�ϴ�������

u8 readflg=0;//��ʱ���ش��Ŀɲ�����־
extern u32 pro_time;//��ʱ���ش��ĳ���������ʱ��
u32 tpro_time=0;//��ʱ������������ʱ��¼��ʱ���ش��ĳ���������ʱ��
u8 time_str[]={"0000"};//��ʱ���飬������Ļ��ʾ����ʱ��

u16 data_reg[30]={0};//ȡƽ��������
float data_reg_f[30]={0.0};//ȡƽ��������
u16 max=0,min=0xffff;//��ʱ����������ȡ��ֵ
float max_f=0,min_f=999999;//��ʱ����������ȡ��ֵ
u32 rev;//��ʱ����������ȡƽ��
double rev_f;//��ʱ����������ȡ��ֵ
u8 data_str[]={"00000.00"};//��ʱ���飬������Ļ��ʾ��������
		
u8 upload_count=0;//���͵������ϵļ���λ
u8 send_count=0;//�ǵ�10����һ��ƽ��ֵ���ϴ�
//���ϱ���Ϊ�������

//-----------------Users application--------------------------
void LoRaWAN_Func_Process(void)
{
    static DEVICE_MODE_T dev_stat = NO_MODE;
    uint16_t temper = 0;
    switch((uint8_t)device_mode)
    {
    /* ָ��ģʽ */
    case CMD_CONFIG_MODE:
    {
        /* �������command Configuration function, �����if���,ִֻ��һ�� */
        if(dev_stat != CMD_CONFIG_MODE)
        {
            dev_stat = CMD_CONFIG_MODE;
            debug_printf("\r\n[Command Mode]\r\n");
						LCD_Fill(48,40,240,52,WHITE);
						LCD_ShowString(48,40,"Command Mode",BLUE);
            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
        }
        /* �ȴ�usart2�����ж� */
        if(UART_TO_PC_RECEIVE_FLAG)
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            lpusart1_send_data(UART_TO_PC_RECEIVE_BUFFER,UART_TO_PC_RECEIVE_LENGTH);
        }
        /* �ȴ�lpuart1�����ж� */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /* ͸��ģʽ */
    case DATA_TRANSPORT_MODE:
    {
        /* �������data transport function,�����if���,ִֻ��һ�� */
        if(dev_stat != DATA_TRANSPORT_MODE)
        {
            dev_stat = DATA_TRANSPORT_MODE;
            debug_printf("\r\n[Transperant Mode]\r\n");
						LCD_Fill(48,40,240,52,WHITE);
						LCD_ShowString(48,40,"Transperant Mode",BLUE);

            /* ģ�������ж� */
            if(nodeJoinNet(JOIN_TIME_120_SEC) == false)
            {
                return;
            }

            temper = HDC1000_Read_Temper()/1000;

            nodeDataCommunicate((uint8_t*)&temper,sizeof(temper),&pphead);
        }

        /* �ȴ�usart2�����ж� */
        if(UART_TO_PC_RECEIVE_FLAG && GET_BUSY_LEVEL)  //Ensure BUSY is high before sending data
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            nodeDataCommunicate((uint8_t*)UART_TO_PC_RECEIVE_BUFFER, UART_TO_PC_RECEIVE_LENGTH, &pphead);
        }

        /* ���ģ����æ, ����������Ч��������������Ϣ */
        else if(UART_TO_PC_RECEIVE_FLAG && (GET_BUSY_LEVEL == 0))
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            debug_printf("--> Warning: Don't send data now! Module is busy!\r\n");
        }

        /* �ȴ�lpuart1�����ж� */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /*����ģʽ*/
    case PRO_TRAINING_MODE:
    {
        /* �������Class C��ƽ̨���ݲɼ�ģʽ, �����if���,ִֻ��һ�� */
        if(dev_stat != PRO_TRAINING_MODE)
        {
            dev_stat = PRO_TRAINING_MODE;
            debug_printf("\r\n[Project Mode]\r\n");
						LCD_Fill(48,40,240,48,WHITE);
						LCD_ShowString(48,40,"Project Mode",BLUE);
						//LCD_ShowString(48,52," ",BLUE);
        }
				
//*****************************************************************************************************************************
//���빤��ģʽ*****************************************************************************************************************
//*****************************************************************************************************************************
				if(settingmod)//������ģʽ��
				{
					ifpoint=0;//��¼�Ƿ��е���������ã�ʵ��û��
					if(Pen_Point.Key_Sta == 1)//�д�������
					{
						ifpoint=1;
							Pen_Point.Key_Sta = 0;
							posx=Pen_Point.X;//��¼���괦����ֹ�������
							posy=Pen_Point.Y;
							if(posx>1400&&posy>1800)//�������Ͻ�
							{
								settingmod=0;//�˳�����ģʽ
								LCD_Fill(0,152,240,320,WHITE);
								LCD_ShowString(8,104,"measured data:",BLUE);
								LCD_ShowString(8,120,"Uploaded data:",BLUE);
								LCD_ShowString(8,150,"states:",BLUE);
								LCD_ShowString(8,170,"call-back data:",BLUE);
							}
							if(posy>1200&&posy<1400)//�޸Ĵ���������
							{
								sensor_type=(sensor_type+1)%4;
			          
								//my_flash_write_uint32_t();
								upload_data[3]=sensor_type+1;
								send_count=0;//�ǵ�10����һ��ƽ��ֵ���ϴ�
								//upload_count=0;//���͵������ϵļ���λ��������������������
								switch(sensor_type)
								{
									case 0:LCD_ShowString(104,84,"Lux",BLUE);break;
									case 1:LCD_ShowString(104,84,"Pressure",BLUE);break;
									case 2:LCD_ShowString(104,84,"Temper",BLUE);break;
									case 3:LCD_ShowString(104,84,"Humidi",BLUE);break;
									default:LCD_ShowString(104,84,"Error",BLUE);break;
								}
							}
							if(posy>850&&posy<1000)//�޸��ϴ�ʱ����������ö�ʱ��tim4��ʱ
							{
								upload_span=upload_span%5+1;//12345���л�
								temp_chr_2[0]=':';
								temp_chr_2[1]=upload_span%10+48;
								LCD_ShowString(96,168,temp_chr_2,BLUE);
							}
							if(posy>500&&posy<800)//�޸��ϴ�����
							{
								upload_times=upload_times%30+5;//5 10 15 20 25 30���л�
								temp_chr_2[0]=upload_times/10%10+48;
								temp_chr_2[1]=upload_times%10+48;
								LCD_ShowString(112,216,temp_chr_2,BLUE);
							}
					}
				}
				else//��ͨģʽ
				{
					ifpoint=0;
					if(Pen_Point.Key_Sta == 1)//�д�������
					{
						ifpoint=1;
							Pen_Point.Key_Sta = 0;
							posx=Pen_Point.X;
							posy=Pen_Point.Y;
							if(posx>1400&&posy>1800)//��������ģʽ
							{
								settingmod=1;
								LCD_Fill(0,152,240,320,WHITE);
								LCD_ShowString(8,168,"upload span:",BLUE);
								LCD_ShowString(8,216,"upload times:",BLUE);
								LCD_ShowString(8,232,"name:HuJie",BLUE);
								LCD_ShowString(8,248,"major:EE",BLUE);
								temp_chr_2[0]=':';
								temp_chr_2[1]=upload_span%10+48;
								LCD_ShowString(96,168,temp_chr_2,BLUE);
								temp_chr_2[0]=upload_times/10%10+48;
								temp_chr_2[1]=upload_times%10+48;
								LCD_ShowString(112,216,temp_chr_2,BLUE);
							}
					}
					if(readflg)//��ʱ����ʱ����һ������(��������1s-5s����)
					{
						readflg=0;
            float temper,humid,pre;
						//LCD_ShowString(0,0,"readflg",BLUE);
						//debug_printf("\r\sensor_type\r\n%d",sensor_type);
						upload_data[3]=sensor_type+1;
						switch(sensor_type)//���ݴ��������Ͷ�����
						{
							case 0:data_reg_f[send_count]=OPT3001_Get_Lux();break;
							case 1:pre=MPL3115_ReadPressure();data_reg_f[send_count]=pre/1000.0;break;
							case 2:temper=HDC1000_Read_Temper();data_reg_f[send_count]=(double)temper/1000.0;break;
							case 3:humid=HDC1000_Read_Humidi();data_reg_f[send_count]=(double)humid/1000.0;break;
							default:break;
						}
						send_count=(send_count+1)%upload_times;//�������
						if(send_count==0)//����ƹ������ˣ���ʼ��ƽ��ֵ
						{
							upload_data[4]=upload_count;//���ϴ������ݵĵ�5λ��ֵ�ϴ�����
							upload_count++;
							if(sensor_type==0||sensor_type==1 || sensor_type==2||sensor_type==3 )//ǰ���ִ������Ǹ�����
							{
								max_f=0;min_f=999999;rev_f=0;
								for(u8 i=0;i<upload_times;i++)//ȡ��ֵ
								{
									if(data_reg_f[i]<min_f) min_f=data_reg_f[i];
									if(data_reg_f[i]>max_f) max_f=data_reg_f[i];
								}
								for(u8 i=0;i<upload_times;i++) rev_f+=data_reg_f[i];//ȫ��������
								rev_f-=min_f;//��ȥ���ֵ����Сֵ
								rev_f-=max_f;
								rev_f/=(upload_times-2);//ȡƽ��ֵ
								debug_printf("data0 = %f\n",rev_f);
								uint16_t x;
								x=((uint16_t)rev_f);
								//��������ʾ��������measured data
								data_str[0]=(u32)rev_f/10000%10+48;
								data_str[1]=(u32)rev_f/1000%10+48;
								data_str[2]=(u32)rev_f/100%10+48;
								data_str[3]=(u32)rev_f/10%10+48;
								data_str[4]=(u32)rev_f%10+48;
								data_str[6]=(u32)(rev_f*10)%10+48;
								data_str[7]=(u32)(rev_f*100)%10+48;
								//�����Ǹ����ϴ�����
								debug_printf("data1 = %d\n",x);
								upload_data[5]=(x>>8)&0xff;
								upload_data[6]=x&0xff;
								debug_printf("data2 = %d %d\n",upload_data[5],upload_data[6]);
								upload_data[7]=((u32)(rev_f*100))%100;
							}
							else if(sensor_type==2||sensor_type==3)//����������
							{
								max=0;min=0xffff;rev=0;
								for(u8 i=0;i<upload_times;i++)
								{
									if(data_reg[i]<min) min=data_reg[i];
									if(data_reg[i]>max) max=data_reg[i];
								}
								for(u8 i=0;i<upload_times;i++) rev+=data_reg[i];
								rev-=min;
								rev-=max;
								rev/=(upload_times-2);
								data_str[0]=(u32)rev/10000%10+48;
								data_str[1]=(u32)rev/1000%10+48;
								data_str[2]=(u32)rev/100%10+48;
								data_str[3]=(u32)rev/10%10+48;
								data_str[4]=(u32)rev%10+48;
								data_str[6]=48;
								data_str[7]=48;
								upload_data[5]=rev>>8&0xff;
								upload_data[6]=rev&0xff;
								upload_data[7]=0x00;
							}
							LCD_ShowString(120,104,data_str,BLUE);//��ʾ������ƽ��ֵ
							////////////�޸� test
							switch(sensor_type)//�ж��Ƿ񳬹���ֵ
							{
								case 0:if(rev_f>300) LEDWAKE_ON;else LEDWAKE_OFF;LEDMODE_OFF;LEDBUSY_OFF;LEDSTAT_OFF;break;
								case 1:if(rev_f>101) LEDBUSY_ON;else LEDBUSY_OFF;LEDWAKE_OFF;LEDMODE_OFF;LEDSTAT_OFF;break;
								case 2:if(rev>30)  LEDMODE_ON;else LEDMODE_OFF;LEDWAKE_OFF;LEDBUSY_OFF;LEDSTAT_OFF;break;
								case 3:if(rev>60) LEDSTAT_ON;else LEDSTAT_OFF;LEDWAKE_OFF;LEDMODE_OFF;LEDBUSY_OFF;break;
								default:LEDWAKE_OFF;LEDMODE_OFF;LEDBUSY_OFF;LEDSTAT_OFF;break;
							}
							LCD_ShowHex();//��16���Ƶ�������ʾ����Ļ�ϣ���Ϊ������ڸ��������½���һ������
							nodeDataCommunicate(upload_data, 9, &pphead);//�ϴ�����
							if (pphead != NULL) // LCD_ShowString(16,200,"no data reveived",BLUE); 
							{
								LCD_Fill(56,170,240,186,WHITE);
								LCD_Fill(0,186,240,320,WHITE);
								LCD_ShowString(56,170,pphead->down_info.business_data,BLUE);//�ж��Ƿ��лش����� 
							}
							//��������
							else LCD_ShowString(16,186,"no data reveived",BLUE);
							//debug_printf("load data = %d %d/n",int_data[0],int_data[1]);
							my_flash_write_uint32_t();
						 // my_flash_read_uint32_t();
							
						}
					}
				}
				/*if(ifpoint)//��ʾ�������
				{
					cnt[0]=posx/10000%10+48;
					cnt[1]=posx/1000%10+48;
					cnt[2]=posx/100%10+48;
					cnt[3]=posx/10%10+48;
					cnt[4]=posx%10+48;
					cnt[6]=posy/10000%10+48;
					cnt[7]=posy/1000%10+48;
					cnt[8]=posy/100%10+48;
					cnt[9]=posy/10%10+48;
					cnt[10]=posy%10+48;
					LCD_ShowString(56,24,cnt,BLUE);
				}*/
				/* ���ʵ�����λ�� */
    }
//*****************************************************************************************************************************
//�˳�����ģʽ*****************************************************************************************************************
//*****************************************************************************************************************************
    break;

    default:
        break;
    }
		if(tpro_time!=pro_time)//����ϵͳ����ʱ��(������һ��)
		{
			time_str[0]=pro_time/1000%10+48;
			time_str[1]=pro_time/100%10+48;
			time_str[2]=pro_time/10%10+48;
			time_str[3]=pro_time%10+48;
			LCD_ShowString(112,24,time_str,BLUE);
			tpro_time=pro_time;
		}
}


/**
 * @brief   ������汾��Ϣ�Ͱ���ʹ��˵����Ϣ��ӡ
 * @details �ϵ����еƻ������100ms
 * @param   ��
 * @return  ��
 */
void LoRaWAN_Borad_Info_Print(void)
{
    debug_printf("\r\n\r\n");
    PRINT_CODE_VERSION_INFO("%s",CODE_VERSION);
    debug_printf("\r\n");
    debug_printf("--> Press Key1 to: \r\n");
    debug_printf("-->  - Enter command Mode\r\n");
    debug_printf("-->  - Enter Transparent Mode\r\n");
    debug_printf("--> Press Key2 to: \r\n");
    debug_printf("-->  - Enter Project Trainning Mode\r\n");
    LEDALL_ON;
    HAL_Delay(100);
    LEDALL_OFF;
}

void LCD_ShowHex()
{
	u8 updata_str[]={"AA F5 9D 01 00 00 00 00 0f"};//��ʱ���飬������Ļ��ʾ�ϴ�������
	updata_str[10]=upload_data[3]+48;//������λ
	updata_str[12]=upload_data[4]>>4&0x0f;
	updata_str[13]=upload_data[4]&0x0f;
	updata_str[15]=upload_data[5]>>4&0x0f;
	updata_str[16]=upload_data[5]&0x0f; 
	updata_str[18]=upload_data[6]>>4&0x0f;
	updata_str[19]=upload_data[6]&0x0f;
	updata_str[21]=upload_data[7]>>4&0x0f;
	updata_str[22]=upload_data[7]&0x0f;
	for(u8 i=12;i<23;i++)
	{
		switch(updata_str[i])
		{
			case 0:updata_str[i]='0';break;
			case 1:updata_str[i]='1';break;
			case 2:updata_str[i]='2';break;
			case 3:updata_str[i]='3';break;
			case 4:updata_str[i]='4';break;
			case 5:updata_str[i]='5';break;
			case 6:updata_str[i]='6';break;
			case 7:updata_str[i]='7';break;
			case 8:updata_str[i]='8';break;
			case 9:updata_str[i]='9';break;
			case 10:updata_str[i]='A';break;
			case 11:updata_str[i]='B';break;
			case 12:updata_str[i]='C';break;
			case 13:updata_str[i]='D';break;
			case 14:updata_str[i]='E';break;
			case 15:updata_str[i]='F';break;
			default:updata_str[i]=' ';break;
		}
	}
//	for (int i=0;i<9;i++)
	int_data[0]=0;
  int_data[0]|=sensor_type<<24;
	int_data[0]|=upload_data[4]<<16;
	int_data[0]|=upload_data[5]<<8;
	int_data[0]|=upload_data[6];
  int_data[1]=upload_data[7];
	LCD_ShowString(16,135,updata_str,BLUE);
	LCD_ShowString(64,150,"successful",BLUE);
}



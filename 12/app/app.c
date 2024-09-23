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

u16 posx=0,posy=0;//暂时储存点的坐标
u8 ifpoint=0;//是否有点按下，调试用
unsigned char cnt[]={"00000 00000"};//显示点的坐标，调试用

//全工程修改过以下地方
//添加了定时器TIM4的相关库(tim.c/tim.h)
//添加了定时器TIM4的中断函数(stm32l4xx_it.c)
//修改了初始化入网的相关函数(lorawan_node_driver.c)
//修改了按键处理函数(key.c)
//修改了触摸屏初始化函数(ST7789v.c)
//修改了总逻辑处理函数(本文件)
//作者：金丁豪
//最后更新时间：2021/6/12/23/06
//以下变量为自行添加
u8 settingmod=0;//是否在设置模式，1表示在
u8 sensor_type=0;//传感器类型，0123代表4种传感器
u8 upload_span=1;//上传间隔/s 1 2 3 4 5
u8 upload_times=5;//平均次数/次 5 10 15 20 25 30
u8 temp_chr_2[]={"00"};//临时数组，用于屏幕显示上传间隔和平均次数
u8 upload_data[9]={0xAA,0xF5,0x6B,0x01,0x00,0x00,0x00,0x00,0x0f};//上传的数据

u8 readflg=0;//定时器回传的可测量标志
extern u32 pro_time;//定时器回传的程序运行总时间
u32 tpro_time=0;//临时变量，用于暂时记录定时器回传的程序运行总时间
u8 time_str[]={"0000"};//临时数组，用于屏幕显示运行时间

u16 data_reg[30]={0};//取平均的数组
float data_reg_f[30]={0.0};//取平均的数组
u16 max=0,min=0xffff;//临时变量，用于取极值
float max_f=0,min_f=999999;//临时变量，用于取极值
u32 rev;//临时变量，用于取平均
double rev_f;//临时变量，用于取极值
u8 data_str[]={"00000.00"};//临时数组，用于屏幕显示测量数据
		
u8 upload_count=0;//发送的数据上的计数位
u8 send_count=0;//记到10计算一次平均值并上传
//以上变量为自行添加

//-----------------Users application--------------------------
void LoRaWAN_Func_Process(void)
{
    static DEVICE_MODE_T dev_stat = NO_MODE;
    uint16_t temper = 0;
    switch((uint8_t)device_mode)
    {
    /* 指令模式 */
    case CMD_CONFIG_MODE:
    {
        /* 如果不是command Configuration function, 则进入if语句,只执行一次 */
        if(dev_stat != CMD_CONFIG_MODE)
        {
            dev_stat = CMD_CONFIG_MODE;
            debug_printf("\r\n[Command Mode]\r\n");
						LCD_Fill(48,40,240,52,WHITE);
						LCD_ShowString(48,40,"Command Mode",BLUE);
            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
        }
        /* 等待usart2产生中断 */
        if(UART_TO_PC_RECEIVE_FLAG)
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            lpusart1_send_data(UART_TO_PC_RECEIVE_BUFFER,UART_TO_PC_RECEIVE_LENGTH);
        }
        /* 等待lpuart1产生中断 */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /* 透传模式 */
    case DATA_TRANSPORT_MODE:
    {
        /* 如果不是data transport function,则进入if语句,只执行一次 */
        if(dev_stat != DATA_TRANSPORT_MODE)
        {
            dev_stat = DATA_TRANSPORT_MODE;
            debug_printf("\r\n[Transperant Mode]\r\n");
						LCD_Fill(48,40,240,52,WHITE);
						LCD_ShowString(48,40,"Transperant Mode",BLUE);

            /* 模块入网判断 */
            if(nodeJoinNet(JOIN_TIME_120_SEC) == false)
            {
                return;
            }

            temper = HDC1000_Read_Temper()/1000;

            nodeDataCommunicate((uint8_t*)&temper,sizeof(temper),&pphead);
        }

        /* 等待usart2产生中断 */
        if(UART_TO_PC_RECEIVE_FLAG && GET_BUSY_LEVEL)  //Ensure BUSY is high before sending data
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            nodeDataCommunicate((uint8_t*)UART_TO_PC_RECEIVE_BUFFER, UART_TO_PC_RECEIVE_LENGTH, &pphead);
        }

        /* 如果模块正忙, 则发送数据无效，并给出警告信息 */
        else if(UART_TO_PC_RECEIVE_FLAG && (GET_BUSY_LEVEL == 0))
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            debug_printf("--> Warning: Don't send data now! Module is busy!\r\n");
        }

        /* 等待lpuart1产生中断 */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /*工程模式*/
    case PRO_TRAINING_MODE:
    {
        /* 如果不是Class C云平台数据采集模式, 则进入if语句,只执行一次 */
        if(dev_stat != PRO_TRAINING_MODE)
        {
            dev_stat = PRO_TRAINING_MODE;
            debug_printf("\r\n[Project Mode]\r\n");
						LCD_Fill(48,40,240,48,WHITE);
						LCD_ShowString(48,40,"Project Mode",BLUE);
						//LCD_ShowString(48,52," ",BLUE);
        }
				
//*****************************************************************************************************************************
//进入工程模式*****************************************************************************************************************
//*****************************************************************************************************************************
				if(settingmod)//在设置模式下
				{
					ifpoint=0;//记录是否有点击，调试用，实际没用
					if(Pen_Point.Key_Sta == 1)//有触屏反馈
					{
						ifpoint=1;
							Pen_Point.Key_Sta = 0;
							posx=Pen_Point.X;//记录坐标处理，防止点击过快
							posy=Pen_Point.Y;
							if(posx>1400&&posy>1800)//点在右上角
							{
								settingmod=0;//退出设置模式
								LCD_Fill(0,152,240,320,WHITE);
								LCD_ShowString(8,104,"measured data:",BLUE);
								LCD_ShowString(8,120,"Uploaded data:",BLUE);
								LCD_ShowString(8,150,"states:",BLUE);
								LCD_ShowString(8,170,"call-back data:",BLUE);
							}
							if(posy>1200&&posy<1400)//修改传感器类型
							{
								sensor_type=(sensor_type+1)%4;
			          
								//my_flash_write_uint32_t();
								upload_data[3]=sensor_type+1;
								send_count=0;//记到10计算一次平均值并上传
								//upload_count=0;//发送的数据上的计数位，换传感器好像不用清零
								switch(sensor_type)
								{
									case 0:LCD_ShowString(104,84,"Lux",BLUE);break;
									case 1:LCD_ShowString(104,84,"Pressure",BLUE);break;
									case 2:LCD_ShowString(104,84,"Temper",BLUE);break;
									case 3:LCD_ShowString(104,84,"Humidi",BLUE);break;
									default:LCD_ShowString(104,84,"Error",BLUE);break;
								}
							}
							if(posy>850&&posy<1000)//修改上传时间间隔，间隔用定时器tim4计时
							{
								upload_span=upload_span%5+1;//12345中切换
								temp_chr_2[0]=':';
								temp_chr_2[1]=upload_span%10+48;
								LCD_ShowString(96,168,temp_chr_2,BLUE);
							}
							if(posy>500&&posy<800)//修改上传个数
							{
								upload_times=upload_times%30+5;//5 10 15 20 25 30中切换
								temp_chr_2[0]=upload_times/10%10+48;
								temp_chr_2[1]=upload_times%10+48;
								LCD_ShowString(112,216,temp_chr_2,BLUE);
							}
					}
				}
				else//普通模式
				{
					ifpoint=0;
					if(Pen_Point.Key_Sta == 1)//有触屏反馈
					{
						ifpoint=1;
							Pen_Point.Key_Sta = 0;
							posx=Pen_Point.X;
							posy=Pen_Point.Y;
							if(posx>1400&&posy>1800)//进入设置模式
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
					if(readflg)//定时器计时到读一次数据(根据设置1s-5s不等)
					{
						readflg=0;
            float temper,humid,pre;
						//LCD_ShowString(0,0,"readflg",BLUE);
						//debug_printf("\r\sensor_type\r\n%d",sensor_type);
						upload_data[3]=sensor_type+1;
						switch(sensor_type)//根据传感器类型读数据
						{
							case 0:data_reg_f[send_count]=OPT3001_Get_Lux();break;
							case 1:pre=MPL3115_ReadPressure();data_reg_f[send_count]=pre/1000.0;break;
							case 2:temper=HDC1000_Read_Temper();data_reg_f[send_count]=(double)temper/1000.0;break;
							case 3:humid=HDC1000_Read_Humidi();data_reg_f[send_count]=(double)humid/1000.0;break;
							default:break;
						}
						send_count=(send_count+1)%upload_times;//计数多次
						if(send_count==0)//如果计够次数了，开始求平均值
						{
							upload_data[4]=upload_count;//给上传的数据的第5位赋值上传次数
							upload_count++;
							if(sensor_type==0||sensor_type==1 || sensor_type==2||sensor_type==3 )//前两种传感器是浮点型
							{
								max_f=0;min_f=999999;rev_f=0;
								for(u8 i=0;i<upload_times;i++)//取极值
								{
									if(data_reg_f[i]<min_f) min_f=data_reg_f[i];
									if(data_reg_f[i]>max_f) max_f=data_reg_f[i];
								}
								for(u8 i=0;i<upload_times;i++) rev_f+=data_reg_f[i];//全部加起来
								rev_f-=min_f;//减去最大值和最小值
								rev_f-=max_f;
								rev_f/=(upload_times-2);//取平均值
								debug_printf("data0 = %f\n",rev_f);
								uint16_t x;
								x=((uint16_t)rev_f);
								//这里是显示测量数据measured data
								data_str[0]=(u32)rev_f/10000%10+48;
								data_str[1]=(u32)rev_f/1000%10+48;
								data_str[2]=(u32)rev_f/100%10+48;
								data_str[3]=(u32)rev_f/10%10+48;
								data_str[4]=(u32)rev_f%10+48;
								data_str[6]=(u32)(rev_f*10)%10+48;
								data_str[7]=(u32)(rev_f*100)%10+48;
								//这里是更新上传数据
								debug_printf("data1 = %d\n",x);
								upload_data[5]=(x>>8)&0xff;
								upload_data[6]=x&0xff;
								debug_printf("data2 = %d %d\n",upload_data[5],upload_data[6]);
								upload_data[7]=((u32)(rev_f*100))%100;
							}
							else if(sensor_type==2||sensor_type==3)//后两种整数
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
							LCD_ShowString(120,104,data_str,BLUE);//显示测量的平均值
							////////////修改 test
							switch(sensor_type)//判断是否超过阈值
							{
								case 0:if(rev_f>300) LEDWAKE_ON;else LEDWAKE_OFF;LEDMODE_OFF;LEDBUSY_OFF;LEDSTAT_OFF;break;
								case 1:if(rev_f>101) LEDBUSY_ON;else LEDBUSY_OFF;LEDWAKE_OFF;LEDMODE_OFF;LEDSTAT_OFF;break;
								case 2:if(rev>30)  LEDMODE_ON;else LEDMODE_OFF;LEDWAKE_OFF;LEDBUSY_OFF;LEDSTAT_OFF;break;
								case 3:if(rev>60) LEDSTAT_ON;else LEDSTAT_OFF;LEDWAKE_OFF;LEDMODE_OFF;LEDBUSY_OFF;break;
								default:LEDWAKE_OFF;LEDMODE_OFF;LEDBUSY_OFF;LEDSTAT_OFF;break;
							}
							LCD_ShowHex();//将16进制的数据显示在屏幕上，因为程序过于复杂于是新建了一个函数
							nodeDataCommunicate(upload_data, 9, &pphead);//上传数据
							if (pphead != NULL) // LCD_ShowString(16,200,"no data reveived",BLUE); 
							{
								LCD_Fill(56,170,240,186,WHITE);
								LCD_Fill(0,186,240,320,WHITE);
								LCD_ShowString(56,170,pphead->down_info.business_data,BLUE);//判断是否有回传数据 
							}
							//下行数据
							else LCD_ShowString(16,186,"no data reveived",BLUE);
							//debug_printf("load data = %d %d/n",int_data[0],int_data[1]);
							my_flash_write_uint32_t();
						 // my_flash_read_uint32_t();
							
						}
					}
				}
				/*if(ifpoint)//显示点的坐标
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
				/* 你的实验代码位置 */
    }
//*****************************************************************************************************************************
//退出工程模式*****************************************************************************************************************
//*****************************************************************************************************************************
    break;

    default:
        break;
    }
		if(tpro_time!=pro_time)//更新系统运行时间(最上面一行)
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
 * @brief   开发板版本信息和按键使用说明信息打印
 * @details 上电所有灯会短暂亮100ms
 * @param   无
 * @return  无
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
	u8 updata_str[]={"AA F5 9D 01 00 00 00 00 0f"};//临时数组，用于屏幕显示上传的数据
	updata_str[10]=upload_data[3]+48;//传感器位
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



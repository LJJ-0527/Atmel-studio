#include <asf.h>
#include "system.h"
#include "lcd.h"
#include "GBK_LibDrive.h"

uint16_t	compressing=0;//启动状态
uint16_t	motor1=0;//固定电机，尖刺电机
uint16_t	zz=0,fz=0,tz=0;//正转，反转，停止
uint16_t	bz1=0,bz2=0;//步骤
uint16_t	reset_flag=0;//判断复位
uint16_t	bz_status=0;//判断步骤
uint16_t	PA11_high = 0;// 是否检测到 PA11 高电平
uint16_t	interrupt_time = 0;//计数
uint16_t    sb=0;//定时逻辑二次判断
uint16_t	ss=0,xj=0;//上升限位，下降限位
uint16_t	sj=0;//升降机构
uint16_t	classify=0;//分类
uint16_t	RV[3];//ADC数据
uint16_t	s=0,y=0;//塑料瓶。易拉罐
uint16_t	has_counted = 0;// 计数标记
uint16_t	target_value=0;//角度值
uint16_t	promote=0;//提升标志位状态

struct tcc_module tcc_instance;
struct adc_module ygy;

void judge(void);
void con_adc(void);
void demould(void);

//端口初始化
void io_init(void)
{
	struct port_config config_port;
	port_get_config_defaults(&config_port);
	config_port.direction=PORT_PIN_DIR_OUTPUT;
	
	//瓶子夹紧电机
	port_pin_set_config(PIN_PA12,&config_port);
	port_pin_set_config(PIN_PA13,&config_port);

	//大电机
	port_pin_set_config(PIN_PB04,&config_port);
	port_pin_set_config(PIN_PB05,&config_port);
	
	//固定电机
	port_pin_set_config(PIN_PA20,&config_port);
	port_pin_set_config(PIN_PA21,&config_port);
	
	//角度电机
	port_pin_set_config(PIN_PA22,&config_port);
	port_pin_set_config(PIN_PA23,&config_port);

	//上升电机
	port_pin_set_config(PIN_PB14,&config_port);
	port_pin_set_config(PIN_PB15,&config_port);

	config_port.direction=PORT_PIN_DIR_INPUT;
	config_port.input_pull=PORT_PIN_PULL_UP;
	
	port_pin_set_config(PIN_PB30,&config_port);			//限位1停止
	port_pin_set_config(PIN_PA10,&config_port);			//限位2夹紧
	port_pin_set_config(PIN_PA11,&config_port);			//限位3松开
	port_pin_set_config(PIN_PB10,&config_port);			//限位4反转
	port_pin_set_config(PIN_PB22,&config_port);			//按键PB22停止
	port_pin_set_config(PIN_PB23,&config_port);			//按键PB23启动
	//port_pin_set_config(PIN_PB,&config_port);			//按键PB22启动提升分类机构模拟小车运行到位
	port_pin_set_config(PIN_PB13,&config_port);			//金属探测
	port_pin_set_config(PIN_PB12,&config_port);			//下面限位
	port_pin_set_config(PIN_PB11,&config_port);			//上面限位
	port_pin_set_config(PIN_PA25,&config_port);			//红外塑料瓶
	port_pin_set_config(PIN_PA24,&config_port);			//红外易拉罐
}

void con_tcc1(void)
{
	struct tcc_config config_tcc;//tcc_config是结构体类型，config_tcc是结构体类型变量
	tcc_get_config_defaults(&config_tcc, TCC1);//调用函数为config——tcc为初始化默认配置

	config_tcc.counter.period = 100000;
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV256;//分频系数

	tcc_init(&tcc_instance, TCC1, &config_tcc);
}

void reset_timer(void)
{
	tcc_clear_status(&tcc_instance, TCC_STATUS_COUNT_OVERFLOW); // 清除溢出标志
	tcc_set_count_value(&tcc_instance, 0); // 重置计数器值
}

int zlz()
{
	if (port_pin_get_input_level(PIN_PA25)==0&&port_pin_get_input_level(PIN_PA24)==0)
	{
		return 1;
	}
	else if (port_pin_get_input_level(PIN_PA25)==1)
	{
		s=0;
		return 0;
	}
	if (port_pin_get_input_level(PIN_PA24)==1)
	{
		y=0;
		return 0;
	}
	return 0;
}

int main (void)
{
	system_init();
	io_init();
	delay_init();
	con_tcc1();
	LCD_Init();           //初始化LCD SPI 接口
	GBK_Lib_Init();       //硬件GBK字库初始化--(如果使用不带字库的液晶屏版本，此处可以屏蔽，不做字库初始化）
	GBK_Show_Str(10,49,240,16,"欢迎来到上海 2023",16,RED,WHITE,0);	//显示内码低字节

	//大电机停止
	port_pin_set_output_level(PIN_PB04,0);
	port_pin_set_output_level(PIN_PB05,0);
	//固定电机停止
	port_pin_set_output_level(PIN_PA20,0);
	port_pin_set_output_level(PIN_PA21,0);
	//port_pin_set_output_level(PIN_PA12,1);
	//port_pin_set_output_level(PIN_PA13,0);
	//delay_ms(1000);
	//port_pin_set_output_level(PIN_PA12,1);
	//port_pin_set_output_level(PIN_PA13,1);
	while (1)
	{
		zlz();
		con_adc();
		interrupt_time = tcc_get_count_value(&tcc_instance);
		//按键按下电机启动
		if (port_pin_get_input_level(PIN_PB23)==0&&reset_flag==0&&promote==1&&zlz()==1)
		{
			delay_ms(5);
			if (port_pin_get_input_level(PIN_PB23)==0&&reset_flag==0&&promote==1&&zlz()==1)
			{
				while (port_pin_get_input_level(PIN_PB23)==0);
				LCD_Clear(WHITE); //清屏
				GBK_Show_Str(60,33,240,16,"启动",16,RED,WHITE,1);	//显示内码低字节
				GBK_Show_Str(60,49,240,16,"压缩中",16,RED,WHITE,1);	//显示内码低字节
				PA11_high = 0;classify=0;
				//大电机停止
				port_pin_set_output_level(PIN_PB04,0);
				port_pin_set_output_level(PIN_PB05,0);
				compressing=1;zz=1;//纸杯，正转
				sb=0;
			}
		}
		//复位按键
		if (port_pin_get_input_level(PIN_PB22)==1&&zlz()==1)
		{
			if (port_pin_get_input_level(PIN_PB23)==0&&reset_flag==1)
			{
				delay_ms(5);
				if (port_pin_get_input_level(PIN_PB23)==0&&reset_flag==1)
				{
					while (port_pin_get_input_level(PIN_PB23)==0);
					reset_flag=0;
					PA11_high = 1;
					if (bz_status==1)
					{
						port_pin_set_output_level(PIN_PB04,1);
						port_pin_set_output_level(PIN_PB05,0);
						// 持续检测PA05的状态
						while (1)
						{
							if (port_pin_get_input_level(PIN_PB30) == 1)
							{
								delay_ms(5);
								if (port_pin_get_input_level(PIN_PB30) == 1)
								{
									bz1=0,bz2=0;//步骤
									port_pin_set_output_level(PIN_PB04, 0);
									port_pin_set_output_level(PIN_PB05, 0);
									bz_status=0;
									promote=0;
									break;  // 跳出循环
								}
							}
						}
					}
					else if (bz_status==2)
					{
						port_pin_set_output_level(PIN_PB04,1);
						port_pin_set_output_level(PIN_PB05,0);
						port_pin_set_output_level(PIN_PA20,0);
						port_pin_set_output_level(PIN_PA21,1);
						delay_ms(390);
						port_pin_set_output_level(PIN_PA20,0);
						port_pin_set_output_level(PIN_PA21,0);
						// 持续检测PA05的状态
						while (1)
						{
							if (port_pin_get_input_level(PIN_PB30) == 1)
							{
								delay_ms(5);
								if (port_pin_get_input_level(PIN_PB30) == 1)
								{
									bz1=0,bz2=0;//步骤
									port_pin_set_output_level(PIN_PB04, 0);
									port_pin_set_output_level(PIN_PB05, 0);
									bz_status=0;
									promote=0;
									break;  // 跳出循环
								}
							}
						}
					}
				}
			}
		}
		//急停按键
		if (port_pin_get_input_level(PIN_PB22)==0&&compressing==1)
		{
			delay_ms(5);
			if (port_pin_get_input_level(PIN_PB22)==0)
			{
				PA11_high=1;
				reset_flag=1;
				compressing=0;
				zz=0,fz=0,tz=0;//正转，反转，停止
				motor1=0;//固定电机，尖刺电机
				tcc_disable(&tcc_instance); // 停止定时器
				tcc_set_count_value(&tcc_instance, 0); // 重置计数器值
				//保存当前步骤状态
				if (bz1) bz_status = 1;
				if (bz2) bz_status = 2;
				bz1=0,bz2=0;
				//大电机停止
				port_pin_set_output_level(PIN_PB04,1);
				port_pin_set_output_level(PIN_PB05,1);
				//固定电机
				port_pin_set_output_level(PIN_PA20,0);
				port_pin_set_output_level(PIN_PA21,0);
			}
		}
		if (compressing==1&&zlz()==1)
		{
			//正转
			if (compressing==1&&zz==1)
			{
				bz1=1;
				port_pin_set_output_level(PIN_PB04,0);
				port_pin_set_output_level(PIN_PB05,1);
			}
			
			//第二个限位检测，第二个电机置1
			if (port_pin_get_input_level(PIN_PA10)==1&&motor1==0)//固定电机
			{
				delay_ms(1);
				if (port_pin_get_input_level(PIN_PA10)==1&&motor1==0)
				{
					bz1=0;
					bz2=1;
					port_pin_set_output_level(PIN_PA20,1);
					port_pin_set_output_level(PIN_PA21,0);
					delay_ms(390);
					port_pin_set_output_level(PIN_PA20,0);
					port_pin_set_output_level(PIN_PA21,0);
					motor1=1;
				}
			}
			//第三个限位检测，电机1置2（不触发限位23函数)
			if (port_pin_get_input_level(PIN_PA11)==1&&motor1==1)//固定电机
			{
				delay_ms(1);
				if (port_pin_get_input_level(PIN_PA11)==1&&motor1==1)
				{
					reset_timer();
					tcc_enable(&tcc_instance);
					bz2=1;
					motor1=2;
					port_pin_set_output_level(PIN_PA20,0);
					port_pin_set_output_level(PIN_PA21,1);
					delay_ms(500);
					port_pin_set_output_level(PIN_PA20,0);
					port_pin_set_output_level(PIN_PA21,0);
				}
			}
			//第四个限位检测，反转
			if (port_pin_get_input_level(PIN_PB10)==1&&fz==0)
			{
				delay_ms(5);
				if (port_pin_get_input_level(PIN_PB10)==1&&fz==0)
				{
					PA11_high=1;
					bz1=1;
					bz2=0;
					fz=1;tz=1;
					zz=0;
					port_pin_set_output_level(PIN_PB04,1);
					port_pin_set_output_level(PIN_PB05,1);
					sb=1;
					delay_ms(500);
					LCD_Clear(WHITE); //清屏
					GBK_Show_Str(60,33,240,16,"压缩结束",16,RED,B_Color,1);	//显示内码低字节
					GBK_Show_Str(60,49,240,16,"反转复位",16,RED,B_Color,1);	//显示内码低字节
					port_pin_set_output_level(PIN_PB04,1);
					port_pin_set_output_level(PIN_PB05,0);
				}
			}
			//第一个限位停止
			if (port_pin_get_input_level(PIN_PB30)==1&&tz==1)
			{
				delay_ms(5);
				if (port_pin_get_input_level(PIN_PB30)==1&&tz==1)
				{
					tz=0;fz=0;zz=0;
					motor1=0;
					compressing=0;
					bz1=0,bz2=0;//步骤
					port_pin_set_output_level(PIN_PB04,1);
					port_pin_set_output_level(PIN_PB05,1);
					tcc_disable(&tcc_instance); // 停止定时器
					tcc_set_count_value(&tcc_instance, 0); // 重置计数器值
					LCD_Clear(WHITE); //清屏
					GBK_Show_Str(60,49,240,16,"压缩完成",16,RED,B_Color,0);	//显示内码低字
					demould();
					promote=0;
				}
			}
			if (classify==1&&tz==1)
			{
				port_pin_set_output_level(PIN_PA22,0);
				port_pin_set_output_level(PIN_PA23,1);
				//判断旋转位置，范围小于100
				if (RV[2]>3600 && RV[2]<3800)
				{
					port_pin_set_output_level(PIN_PA22,1);
					port_pin_set_output_level(PIN_PA23,1);
					classify=0;
				}
			}
			else if (classify==0&&tz==1)
			{
				port_pin_set_output_level(PIN_PA22,1);
				port_pin_set_output_level(PIN_PA23,0);
				//判断旋转位置，范围小于100
				if (RV[2]>250 && RV[2]<400)
				{
					port_pin_set_output_level(PIN_PA22,1);
					port_pin_set_output_level(PIN_PA23,1);
				}
			}
		}
		if (interrupt_time>=30000&&sb==0)
		{
			// 如果到期时没有检测到 PA11 高电平，反转电机
			if (PA11_high==0)
			{
				LCD_Clear(WHITE); //清屏
				GBK_Show_Str(60,49,240,16,"定时复位",16,RED,WHITE,1);	//显示内码低字节
				bz1=0;
				bz2=0;
				fz=1;
				tz=1;
				zz=0;
				PA11_high=1;
				promote=0;
				delay_ms(375);
				port_pin_set_output_level(PIN_PB04,1);
				port_pin_set_output_level(PIN_PB05,0);
				tcc_disable(&tcc_instance); // 停止定时器
				tcc_set_count_value(&tcc_instance, 0); // 重置计数器值
			}
		}
		if (port_pin_get_input_level(PIN_PB23)==0&&promote==0&&zlz()==1)
		{
			delay_ms(5);
			if (port_pin_get_input_level(PIN_PB23)==0&&promote==0&&zlz()==1)
			{
				sj=1;
			}
		}
		if (sj==1)
		{
			if (sj==1&&ss==0)//夹紧电机先启动，上升置1
			{
				ss=1;
				//夹紧电机
				port_pin_set_output_level(PIN_PA12,0);
				port_pin_set_output_level(PIN_PA13,1);
				delay_ms(2000);
				port_pin_set_output_level(PIN_PA12,1);
				port_pin_set_output_level(PIN_PA13,1);
				//上升电机
				port_pin_set_output_level(PIN_PB14,1);
				port_pin_set_output_level(PIN_PB15,0);
			}
			if (port_pin_get_input_level(PIN_PB11)==1&&ss==1)
			{
				delay_ms(5);
				if (port_pin_get_input_level(PIN_PB11)==1&&ss==1)//检测上升置1，到达上方位置PA15读取高电平
				{
					ss=2;
					//上升电机
					port_pin_set_output_level(PIN_PB14,1);
					port_pin_set_output_level(PIN_PB15,1);
					delay_ms(100);
					//夹紧电机反转松瓶子
					port_pin_set_output_level(PIN_PA12,1);
					port_pin_set_output_level(PIN_PA13,0);
					delay_ms(500);
					//上升电机下降
					port_pin_set_output_level(PIN_PB14,0);
					port_pin_set_output_level(PIN_PB15,1);
					xj=1;
				}
			}
			if (port_pin_get_input_level(PIN_PB12)==1&&xj==1)
			{
				delay_ms(5);
				if (port_pin_get_input_level(PIN_PB12)==1&&xj==1)//到达下降位置PA14读取高电平
				{
					xj=0;sj=0;ss=0;
					//上升电机
					port_pin_set_output_level(PIN_PB14,1);
					port_pin_set_output_level(PIN_PB15,1);
					delay_ms(1500);
					//夹紧电机
					port_pin_set_output_level(PIN_PA12,1);
					port_pin_set_output_level(PIN_PA13,1);
					judge();
				}
			}
		}
	}
}



void judge(void)
{
	if (port_pin_get_input_level(PIN_PB13)==1)
	{
		GBK_Show_Str(60,65,240,16,"塑料瓶",16,RED,WHITE,1);	//显示内码低字节
		s++;
		classify=1;
		promote=1;
	}
	else
	{
		GBK_Show_Str(60,65,240,16,"易拉罐",16,RED,WHITE,1);	//显示内码低字节
		y++;
		classify=0;
		promote=1;
	}
}


// ADC 转换函数
void con_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);

	config_adc.reference = ADC_REFERENCE_INTVCC2;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc.resolution = ADC_RESOLUTION_12BIT;

	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN14;
	adc_init(&ygy, ADC, &config_adc);
	adc_enable(&ygy);
	uint16_t value = 0;
	uint32_t sum = 0;
	for (uint8_t i = 0; i < 5; i++)
	{
		adc_start_conversion(&ygy);
		while (adc_read(&ygy, &value) != STATUS_OK);
		sum += value;
	}
	RV[2] = sum / 5;  // 取平均值，减少抖动
	adc_disable(&ygy);
}

void demould(void)
{
	port_pin_set_output_level(PIN_PB04,1);
	port_pin_set_output_level(PIN_PB05,0);
	delay_ms(300);
	port_pin_set_output_level(PIN_PB04,1);
	port_pin_set_output_level(PIN_PB05,1);
}


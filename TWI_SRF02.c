#include <mega128.h>
#include <stdio.h>
#include <delay.h>
#include "lcd.h"
#include "twi.h"

#define COMMAND_REGISTER 0

#define SRF02_Return_inch 80
#define SRF02_Return_Cm 81
#define SRF02_Return_microSecond 82

#define SRF02_1st_Sequence_change 160
#define SRF02_2nd_Sequence_change 170
#define SRF02_3rd_Sequence_change 165

#define ExtDev_ERR_CNT_MAX 2000

/*
  * @brief �����ļ������� ����� ���� I2C �ʱ�ȭ �Լ� 
*/
void I2C_Init(void)
{
    TWBR = 0x40; // 100kHz I2C clock frequency
}

/**
  * @brief  I2C �� ���� ������ ���� �Լ� 
  * @param  address: I2C�� ���� ������ ���� ��� ��ġ �ּ�
  * @param  reg: I2C�� ���� ������ ���� �������� �ּ�
  * @param  data: I2C�� ���� ������ ���� �������� ��
  * @retval �۽� ���� ���� ( 0: ����, 1 : ����)
  */
unsigned char SRF02_I2C_Write(char address, char reg, char data)
{ 
    unsigned char ret_err=0;     
    
    ret_err = TWI_Start();              // I2C ���ۺ�Ʈ ����  
            
    ret_err = TWI_Write_SLAW(address); 	// SLAW ��Ŷ ����  
    if(ret_err != 0) return ret_err;    // error�� ����  
    ret_err = TWI_Write_Data(reg);      // �������� ��ġ �۽�  
    if(ret_err != 0) return ret_err;    // error�� ���� 
    ret_err = TWI_Write_Data(data);     // ���(command) �۽�  
    if(ret_err != 0) return ret_err;    // error�� ���� 
    TWI_Stop();                         // I2C �����Ʈ ���� 
     
    return 0;                           // ���� ����                                
}

/**
  * @brief  I2C�� ���� ������ read �Լ� 
  * @param address: I2C�� ���� ������ �б� ��� ��ġ �ּ�
  * @param reg: I2C�� ���� ������ �б� ��� �������� �ּ�
  * @retval I2C�� ���� �о��� 1Byte ������ 
  */
unsigned char SRF02_I2C_Read(char address, char reg, unsigned char* Data)
{                
    char read_data = 0;    
    unsigned char ret_err=0;   
    char str[10];    
        
    ret_err = TWI_Start();        

    ret_err = TWI_Write_SLAW(address);  // SLAW ��Ŷ ����
    if(ret_err != 0) return ret_err;    // error�� ����   
    
    ret_err = TWI_Write_Data(reg);      // �������� ��ġ �۽� 
    if(ret_err != 0) return ret_err;    // error�� ����  
    
    ret_err = TWI_Restart();            // Restart ��Ʈ ���� 
    PORTB |= 0x08;                      // error�� ���� Ȯ��
    if(ret_err != 0) return ret_err;    // error�� ����  
    
    ret_err = TWI_Write_SLAR(address);  // SLAR ��Ŷ ����   
    PORTB |= 0x10;                      // error�� ���� Ȯ��
    if(ret_err != 0) return ret_err;    // error�� ���� 
    
    ret_err = TWI_Read_Data_Aak(&read_data); // �������� ������ ����
    PORTB |= 0x20;                      // error�� ���� Ȯ��
    if(ret_err != 0) return ret_err;    // error�� ���� 
    
    TWI_Stop();                         // STOP ��ȣ �۽�    
     
    *Data = read_data;
    
    return 0;                           // ���� ���� 
}

/**
  * @brief  �����ļ����� ���� ������ �����ϴ� �Լ� 
  * @param  addr: ������ �Ÿ��� ����� �ϴ� �����ļ����� ��ġ �ּ�
  */
void startRanging(char addr)
{
    // Cm ������ ���� ��û.
    SRF02_I2C_Write(addr, COMMAND_REGISTER, SRF02_Return_Cm);
}

/**
  * @brief  ������ �Ÿ��� �о���� �Լ� 
  * @param  addr: ������ �Ÿ��� ����� �ϴ� �����ļ����� ��� �ּ�
  * @retval �����ļ����� ������ �Ÿ�  
  */
unsigned int getRange(char addr)
{
    unsigned char temp;
    unsigned int x;
    unsigned char res = 0;          
    
    // Get high and then low bytes of the range
    res = SRF02_I2C_Read(addr,2,&temp);             // Get high Byte
    if(res) return res;
    x = temp<<8;
    res = SRF02_I2C_Read(addr,3,&temp);             // Get low Byte  
    if(res) return res;  
    x |= temp;  
    
    return (x);
}

 /**
  * @brief  �����ļ����� ��ġ �ּҸ� �����ϴ� �Լ� 
  * @param  ori: �����ϰ��� �ϴ� �����ļ��� ���� ��ġ �ּ�
  * @param  des: �����ϰ��� �ϴ� �����ļ��� �ű� ��ġ �ּ�
  */
void change_Sonar_Addr(unsigned char ori, unsigned char des)
{
    // ��巹���� �Ʒ��� 16���� ����
    switch(des)
    {
        case 0xE0:
        case 0xE2:
        case 0xE4:
        case 0xE6:
        case 0xE8:
        case 0xEA:
        case 0xEC:
        case 0xEE:
        case 0xF0:
        case 0xF2:
        case 0xF4:
        case 0xF6:
        case 0xF8:
        case 0xFA:
        case 0xFC:
        case 0xFE:
            // ID���� �������� ���� ���� ������ �������� ���� 
            SRF02_I2C_Write(ori, COMMAND_REGISTER, SRF02_1st_Sequence_change);
            SRF02_I2C_Write(ori, COMMAND_REGISTER, SRF02_2nd_Sequence_change);
            SRF02_I2C_Write(ori, COMMAND_REGISTER, SRF02_3rd_Sequence_change);

            // ID���� �������� ���� �ű� ID ����
            SRF02_I2C_Write(ori, COMMAND_REGISTER, des);
            break;
    }
}

unsigned char ti_Cnt_1ms; // 1ms ���� �ð� ��� ���� ���� ��������    
unsigned char LCD_DelCnt_1ms;

/**
  * @brief  1ms ����� ���� Ÿ�̸� ���� 
            �� 1ms(@ 14.7456Mhz)
  */
void Timer0_Init(){
    TCCR0 = (1<<WGM01)|(1<<CS00)|(1<<CS01)|(1<<CS02); //CTC���, 1024����
    TCNT0 = 0x00;
    OCR0  = 14; //14.7456Mhz / 1024���� / 14�ܰ� = 1.028kHz 
    TIMSK = (1<<OCIE0);// ����ġ ���ͷ�Ʈ �㰡   
}

/**
  * @brief  Ÿ�̸�0 ����ġ ���ͷ�Ʈ 
  */
interrupt[TIM0_COMP] void timer0_comp(void)
{                
    ti_Cnt_1ms++;      
    LCD_DelCnt_1ms++;
}

/**
  * @brief  1���� SRF02 �Ÿ������� LCD�� ����ϴ� ���� ���α׷�
  */
void main(void)
{
    char Sonar_addr = 0xE0;      // �����ϰ��� �ϴ� ��ġ �ּ�
    unsigned int Sonar_range;    // ���� �Ÿ��� ������ ����
    char Message[40];            // LCD ȭ�鿡 ���ڿ� ����� ���� ���ڿ� ����       
    int readCnt = 0;             // LCD ȭ�鿡 ���������� ��Ÿ���� ���� ����   
    
    DDRD |= 0x03;
    
    LCD_PORT_Init();                     // LCD ��Ʈ ���� 
    LCD_Init();                     // LCD �ʱ�ȭ   
    
    Timer0_Init();                  // 1ms ��� ���� Ÿ�̸� �ʱ�ȭ 
    I2C_Init();                      // I2C ��� �ʱ�ȭ( baudrate ����)    

    delay_ms(1000);                // SRF02 ���� ����ȭ �ð� ���     

    SREG|=0x80;                   // Ÿ�̸� ���ͷ�Ʈ Ȱ��ȭ ���� ���� ���ͷ�Ʈ Ȱ��ȭ 

    startRanging(Sonar_addr);     // ������ ���� �Ÿ� ���� ���� ���    
    
    ti_Cnt_1ms = 0;               //  �����ð� ��⸦ ���� ���� �ʱ�ȭ     
    
    LCD_DelCnt_1ms = 0;            //LCD ǥ�� �ֱ� ���� ī��    
    
//    delay_ms(500);
//    Sonar_range = getRange(Sonar_addr);    
        
    while(1)
    {
        // ���ð��� 66ms �̻��� ��� 
        if(ti_Cnt_1ms > 66)
        {                                
            // ������ ���� �Ÿ� ���� ������ ���
            Sonar_range = getRange(Sonar_addr);
            
            // lcd ��� ������Ʈ �ֱ� ����            
            if(LCD_DelCnt_1ms > 100)
            {
                // ������ �Ÿ� LCD ȭ�鿡 ���
//                LCD_Pos(0,0);
//                LCD_Str("Measured Dist.= ");

                sprintf(Message, "Distance %03d cm",Sonar_range);
                LCD_Pos(1,0);
                LCD_Str(Message); 
                  
                LCD_DelCnt_1ms = 0;      
            }

            // ������ ���� �Ÿ� ���� ���� ��� 
            startRanging(Sonar_addr);             
            
            // ���ð� �ʱ�ȭ  
            ti_Cnt_1ms = 0; 
            readCnt = (readCnt+1)%10;     
            
        }
    }
}
  
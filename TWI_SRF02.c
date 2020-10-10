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
  * @brief 초음파센서와의 통신을 위한 I2C 초기화 함수 
*/
void I2C_Init(void)
{
    TWBR = 0x40; // 100kHz I2C clock frequency
}

/**
  * @brief  I2C 를 통한 데이터 쓰기 함수 
  * @param  address: I2C를 통한 데이터 쓰기 대상 장치 주소
  * @param  reg: I2C를 통한 데이터 쓰기 레지스터 주소
  * @param  data: I2C를 통한 데이터 쓰기 레지스터 값
  * @retval 송신 성공 여부 ( 0: 실패, 1 : 성공)
  */
unsigned char SRF02_I2C_Write(char address, char reg, char data)
{ 
    unsigned char ret_err=0;     
    
    ret_err = TWI_Start();              // I2C 시작비트 전송  
            
    ret_err = TWI_Write_SLAW(address); 	// SLAW 패킷 전송  
    if(ret_err != 0) return ret_err;    // error시 종료  
    ret_err = TWI_Write_Data(reg);      // 레지스터 위치 송신  
    if(ret_err != 0) return ret_err;    // error시 종료 
    ret_err = TWI_Write_Data(data);     // 명령(command) 송신  
    if(ret_err != 0) return ret_err;    // error시 종료 
    TWI_Stop();                         // I2C 종료비트 전송 
     
    return 0;                           // 정상 종료                                
}

/**
  * @brief  I2C를 통한 데이터 read 함수 
  * @param address: I2C를 통한 데이터 읽기 대상 장치 주소
  * @param reg: I2C를 통한 데이터 읽기 대상 레지스터 주소
  * @retval I2C를 통해 읽어진 1Byte 데이터 
  */
unsigned char SRF02_I2C_Read(char address, char reg, unsigned char* Data)
{                
    char read_data = 0;    
    unsigned char ret_err=0;   
    char str[10];    
        
    ret_err = TWI_Start();        

    ret_err = TWI_Write_SLAW(address);  // SLAW 패킷 전송
    if(ret_err != 0) return ret_err;    // error시 종료   
    
    ret_err = TWI_Write_Data(reg);      // 레지스터 위치 송신 
    if(ret_err != 0) return ret_err;    // error시 종료  
    
    ret_err = TWI_Restart();            // Restart 비트 전송 
    PORTB |= 0x08;                      // error시 상태 확인
    if(ret_err != 0) return ret_err;    // error시 종료  
    
    ret_err = TWI_Write_SLAR(address);  // SLAR 패킷 전송   
    PORTB |= 0x10;                      // error시 상태 확인
    if(ret_err != 0) return ret_err;    // error시 종료 
    
    ret_err = TWI_Read_Data_Aak(&read_data); // 레지스터 데이터 수신
    PORTB |= 0x20;                      // error시 상태 확인
    if(ret_err != 0) return ret_err;    // error시 종료 
    
    TWI_Stop();                         // STOP 신호 송신    
     
    *Data = read_data;
    
    return 0;                           // 전상 종료 
}

/**
  * @brief  초음파센서의 측정 시작을 설정하는 함수 
  * @param  addr: 측정된 거리를 얻고자 하는 초음파센서의 장치 주소
  */
void startRanging(char addr)
{
    // Cm 단위로 측정 요청.
    SRF02_I2C_Write(addr, COMMAND_REGISTER, SRF02_Return_Cm);
}

/**
  * @brief  측정된 거리를 읽어오는 함수 
  * @param  addr: 측정된 거리를 얻고자 하는 초음파센서의 장비 주소
  * @retval 초음파센서의 측정된 거리  
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
  * @brief  초음파센서의 장치 주소를 변경하는 함수 
  * @param  ori: 변경하고자 하는 초음파센서 기존 장치 주소
  * @param  des: 변경하고자 하는 초음파센서 신규 장치 주소
  */
void change_Sonar_Addr(unsigned char ori, unsigned char des)
{
    // 어드레스는 아래의 16개만 허용됨
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
            // ID변경 시퀀스에 따라 기존 센서에 변경명령을 전송 
            SRF02_I2C_Write(ori, COMMAND_REGISTER, SRF02_1st_Sequence_change);
            SRF02_I2C_Write(ori, COMMAND_REGISTER, SRF02_2nd_Sequence_change);
            SRF02_I2C_Write(ori, COMMAND_REGISTER, SRF02_3rd_Sequence_change);

            // ID변경 시퀀스에 따라 신규 ID 전달
            SRF02_I2C_Write(ori, COMMAND_REGISTER, des);
            break;
    }
}

unsigned char ti_Cnt_1ms; // 1ms 단위 시간 계수 위한 전역 변수선언    
unsigned char LCD_DelCnt_1ms;

/**
  * @brief  1ms 계수를 위한 타이머 설정 
            약 1ms(@ 14.7456Mhz)
  */
void Timer0_Init(){
    TCCR0 = (1<<WGM01)|(1<<CS00)|(1<<CS01)|(1<<CS02); //CTC모드, 1024분주
    TCNT0 = 0x00;
    OCR0  = 14; //14.7456Mhz / 1024분주 / 14단계 = 1.028kHz 
    TIMSK = (1<<OCIE0);// 비교일치 인터럽트 허가   
}

/**
  * @brief  타이머0 비교일치 인터럽트 
  */
interrupt[TIM0_COMP] void timer0_comp(void)
{                
    ti_Cnt_1ms++;      
    LCD_DelCnt_1ms++;
}

/**
  * @brief  1개의 SRF02 거리정보를 LCD에 출력하는 메인 프로그램
  */
void main(void)
{
    char Sonar_addr = 0xE0;      // 측정하고자 하는 장치 주소
    unsigned int Sonar_range;    // 측정 거리를 저장할 변수
    char Message[40];            // LCD 화면에 문자열 출력을 위한 문자열 변수       
    int readCnt = 0;             // LCD 화면에 동작중임을 나타내기 위한 변수   
    
    DDRD |= 0x03;
    
    LCD_PORT_Init();                     // LCD 포트 설정 
    LCD_Init();                     // LCD 초기화   
    
    Timer0_Init();                  // 1ms 계수 위한 타이머 초기화 
    I2C_Init();                      // I2C 통신 초기화( baudrate 설정)    

    delay_ms(1000);                // SRF02 전원 안정화 시간 대기     

    SREG|=0x80;                   // 타이머 인터럽트 활성화 위한 전역 인터럽트 활성화 

    startRanging(Sonar_addr);     // 초음파 센서 거리 측정 시작 명령    
    
    ti_Cnt_1ms = 0;               //  측정시간 대기를 위한 변수 초기화     
    
    LCD_DelCnt_1ms = 0;            //LCD 표시 주기 설정 카운    
    
//    delay_ms(500);
//    Sonar_range = getRange(Sonar_addr);    
        
    while(1)
    {
        // 대기시간이 66ms 이상일 경우 
        if(ti_Cnt_1ms > 66)
        {                                
            // 초음파 센서 거리 측정 데이터 얻기
            Sonar_range = getRange(Sonar_addr);
            
            // lcd 출력 업데이트 주기 설정            
            if(LCD_DelCnt_1ms > 100)
            {
                // 측정된 거리 LCD 화면에 출력
//                LCD_Pos(0,0);
//                LCD_Str("Measured Dist.= ");

                sprintf(Message, "Distance %03d cm",Sonar_range);
                LCD_Pos(1,0);
                LCD_Str(Message); 
                  
                LCD_DelCnt_1ms = 0;      
            }

            // 초음파 센서 거리 측정 시작 명령 
            startRanging(Sonar_addr);             
            
            // 대기시간 초기화  
            ti_Cnt_1ms = 0; 
            readCnt = (readCnt+1)%10;     
            
        }
    }
}
  
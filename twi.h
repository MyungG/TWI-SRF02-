#ifndef _INCLUDE_TWI_H__
#define _INCDLUE_TWI_H__

//////////////////////////////

#define ExtDev_ERR_MAX_CNT 2000

// TWI Master Transmitter/Receiver Mode������ ���� �ڵ�     
#define TWSR_TWI_START   0x08 
#define TWSR_TWI_RESTART 0x10
#define MT_SLA_ACK 0x18  
#define MT_DATA_ACK 0x28 
#define MR_SLA_ACK 0x40
#define MR_DATA_ACK 0x58

// TWI Slave Receiver Mode������ ���� �ڵ�  
#define SR_SLA_ACK 0x60 
#define SR_STOP 0xA0    
#define SR_DATA_ACK 0x80         

// TWI Init.
void Init_TWI()
{
    TWBR = 0x32;        //SCL = 100kHz
    TWCR = (1<<TWEN);   //TWI Enable
    TWSR = 0x00;        //100kHz
}

/*****************************************************/
/*         ������ �۽ű� ��忡���� �۽� ���� �Լ�             */
/*****************************************************/

// ��ȣ ���� �Ϸ� �˻� �� Status Ȯ�� + Timeout Check 
// error code 0 : no error, 1 : timeout error 2 : TWI Status error
// 
unsigned char TWI_TransCheck_ACK(unsigned char Stat)
{ 
    char str[10];
    unsigned int ExtDev_ErrCnt = 0;
    while(!(TWCR & (1<<TWINT)))         // ��Ŷ ���� �Ϸ�� �� ���� wait
    { 
        if(ExtDev_ErrCnt++ > ExtDev_ERR_MAX_CNT){ return 1; }
    }      
    
    if((TWSR & 0xf8) != Stat)
    {                    

        LCD_Pos(0,0);

        sprintf(str, "%02x, %02x", TWSR & 0xf8, Stat);     
        LCD_Str(str);
                    
        return 2;  // ���� �˻�(ACK) : error�� 1 ��ȯ  
    } 
        
    else return 0;       
}

// START ���� 
unsigned char TWI_Start()
{
    TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));   // START ��ȣ ������
    while(!(TWCR & (1<<TWINT)));                    // START ��ȣ ���� �Ϸ�� �� ���� wait    
} 
        
// SLA+W ��Ŷ ����
unsigned char TWI_Write_SLAW(unsigned char Addr)
{
    unsigned char ret_err=0;
    TWDR = Addr & 0xFE;                 // SLA + W ��Ŷ(�����̺� �ּ�+Write bit(Low))
    TWCR = (1<<TWINT) | (1<<TWEN);      // SLA + W ��Ŷ ������       
    return TWI_TransCheck_ACK(MT_SLA_ACK); 
} 

// ������ ��Ŷ ����
unsigned char TWI_Write_Data(unsigned char Data)
{
    unsigned char ret_err=0;
    TWDR = Data;                        // ������ 
    TWCR = (1<<TWINT) | (1<< TWEN);     // ������ ��Ŷ �۽�  
    return TWI_TransCheck_ACK(MT_DATA_ACK); 
} 

// STOP ���� 
void TWI_Stop()
{
    TWCR = ((1<<TWINT) | (1<<TWSTO) | (1<<TWEN));   // STOP ��ȣ ������
} 

// RESTART ���� 
unsigned char TWI_Restart()
{
    unsigned char ret_err=0;
    TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));   // Restart ��ȣ ������ 
    return TWI_TransCheck_ACK(TWSR_TWI_RESTART);                 
} 
                      
// Write Packet function for Master 
unsigned char TWI_Master_Transmit(unsigned char Data, unsigned char Addr)  
{
    unsigned char ret_err=0;
    ret_err = TWI_Start();      // START ��ȣ �۽� 
    if(ret_err != 0) return ret_err;  // error�� ���� 
    ret_err = TWI_Write_SLAW(Addr);    // �����̺� �ּ� �۽�  
    if(ret_err != 0) return ret_err;    
    ret_err = TWI_Write_Data(Data);    // ������ �۽� 
    if(ret_err != 0) return ret_err;
    TWI_Stop();                 // STOP ��ȣ �۽�
}

/*****************************************************/
/*  ������ ���ű� ��忡���� �۽� ���� �Լ�  */
/*****************************************************/            
// SLA+R ��Ŷ ����
unsigned char TWI_Write_SLAR(unsigned char Addr)
{
    unsigned char ret_err=0;
    TWDR = Addr|0x01;               // SLA + R ��Ŷ(�����̺� �ּ�+Read bit(High))
    TWCR = (1<<TWINT) | (1<<TWEN);  // SLA + R ��Ŷ ������  
    return TWI_TransCheck_ACK(MR_SLA_ACK);     
}                                

// ������ ��Ŷ ����
unsigned char TWI_Read_Data(unsigned char* Data)
{   
    unsigned char ret_err=0;              
    
    TWCR = (1<<TWINT)|(1<< TWEN);   // SLA + W ��Ŷ ������ 
    ret_err = TWI_TransCheck_ACK(MR_SLA_ACK); 
    
    if(ret_err != 0) 
        return ret_err;         // if error, return error code
    else         
        *Data = TWDR;           // no error, return ���� ������(�����ͷ�)
        
    return 0;                   // ���� ���� 
} 

unsigned char TWI_Read_Data_Aak(unsigned char* Data)
{   
    unsigned char ret_err=0;
    TWCR = (1<<TWINT)|(1<< TWEN);   // SLA + W ��Ŷ ������  
    ret_err = TWI_TransCheck_ACK(MR_DATA_ACK); 
    *Data = TWDR;           // no error, return ���� ������(�����ͷ�)
    return 0;               // ���� ���� 
} 

// Read Packet function for Master 
unsigned char TWI_Master_Receive(unsigned char Addr, unsigned char* Data)  
{
    unsigned char rec_data;
    unsigned char ret_err=0;
    ret_err = TWI_Start();            // START ��ȣ �۽�    
    if(ret_err != 0) return ret_err;  // error�� ����     
    ret_err = TWI_Write_SLAR(Addr);   // �����̺� �ּ� �۽�  
    if(ret_err != 0) return ret_err;  // error�� ���� 
    ret_err = TWI_Read_Data(&rec_data); // �����ͼ��� 
    if(ret_err != 0) return ret_err;  // error�� ���� 
    TWI_Stop();             // STOP ��ȣ �۽�                          
    *Data = rec_data;
    return 0;               // ���� ��
}


/********************************************************/
/*  �����̺� ���ű� ��忡���� ���� ���� �Լ�    */
/********************************************************/
         
// Slave �ּ� ���� �Լ� 
void Init_TWI_Slaveaddr(unsigned char Slave_Addr)
{ 
    TWAR = Slave_Addr;
}
 
// SLA ��Ŷ�� ���� ACK ���� �Լ�
unsigned char TWI_Slave_Match_ACK()
{
    unsigned char ret_err=0; 
    TWCR = ((1<<TWINT)|(1<<TWEA) |(1<<TWEN));
    // ACK ���� ��� Ȱ��ȭ       
    return TWI_TransCheck_ACK(SR_SLA_ACK);  
    // ��Ŷ ���� �Ϸ� ��� �� SLA + W ��Ŷ�� ���� ACK Ȯ��  
} 

// STOP ���� ���� �� ACK ���� �Լ�
unsigned char TWI_Slave_Stop_ACK()
{ 
    unsigned char ret_err=0;
    TWCR = ((1<<TWINT)|(1<<TWEA) |(1<<TWEN));
    // ACK ���� ��� Ȱ��ȭ 
    return TWI_TransCheck_ACK(SR_STOP); 
    // STOP ��ȣ ���� ���           
} 

// ������ ���� �Լ� 
unsigned char TWI_Slave_Read_Data(unsigned char* Data)
{ 
    unsigned char ret_err=0;
    TWCR = ((1<<TWINT)|(1<<TWEA) |(1<<TWEN));
    // ACK ���� ��� Ȱ��ȭ        
    ret_err = TWI_TransCheck_ACK(SR_DATA_ACK);   
    if(ret_err != 0) 
        return ret_err;     // if error, return error code         
    *Data = TWDR;           // no error, return ���� ������ 
    return 0;               // ���� ���� 
} 

// Read Packet function for Slave 
unsigned char TWI_Slave_Receive(unsigned char* Data)
{
    unsigned char ret_err=0;
    unsigned char rec_data;
    ret_err = TWI_Slave_Match_ACK();   
    if(ret_err != 0) return ret_err;  // error�� ����
    ret_err = TWI_Slave_Read_Data(&rec_data); 
    if(ret_err != 0) return ret_err;  // error�� ����    
    ret_err = TWI_Slave_Stop_ACK(); 
    if(ret_err != 0) return ret_err;  // error�� ����     
    *Data = rec_data;           // ���� ������ ��ȯ 
    return 0;	                // ���� ����
}


/**********************************************************/
/* Master TX/RX Mixed function like EEPROM, Sonar etc     */
/**********************************************************/       

unsigned char TWI_Master_Receive_ExDevice(unsigned char devAddr,unsigned char regAddr, unsigned char* Data)  
{
    unsigned char rec_data; 
    unsigned char ret_err=0;
    ret_err = TWI_Start();      // START ��ȣ �۽�    
    if(ret_err != 0) return ret_err;  // error�� ���� 
    ret_err = TWI_Write_SLAW(devAddr);    // �����̺� �ּ� �۽�
    if(ret_err != 0) return ret_err;  // error�� ���� 
    ret_err = TWI_Write_Data(regAddr);    // ���������ּ� �۽�  
    if(ret_err != 0) return ret_err;  // error�� ����             
    ret_err = TWI_Restart();              // Restart �۽� 
    if(ret_err != 0) return ret_err;  // error�� ����     
    ret_err = TWI_Write_SLAR(devAddr);    // �����̺� �������� �ּ� �۽� 
    if(ret_err != 0) return ret_err;  // error�� ����        

    ret_err = TWI_Read_Data(&rec_data); // �������� ������ ����(�ּ� ����)      
    if(ret_err != 0) return ret_err;  // error�� ����      
    TWI_Stop();                 // STOP ��ȣ �۽�
    *Data = rec_data; 
    return 0;
}  

#endif 
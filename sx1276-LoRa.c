/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */

#include  <iostm8s103f3.h>
#include "My_type.h"
#include "sx1276-LoRa.h"

u8 gb_SF;
u8  gb_BW;
u8  CR;	//LR_RegModemConfig1 
  
#define CRC   0x01  //CRC Enable

#define datlen 11

u8 gtmp;
/**********************************************************
**Parameter table define
**********************************************************/
//__root const u16 SX1276FreqTbl[3] = {0x066C, 0x0780, 0x0800}; //434MHz
__root const u16 SX1276FreqTbl[3] = {0x0685, 0x073b, 0x0813}; //434MHz @ 26m
//__root const u16 SX1276FreqTbl[3] = {0x0634, 0x0700, 0x0800}; //169MHz @ 26m

__root const u16 SX1276PowerTbl[4] =
{ 
  0x09FF,                   //20dbm  
  0x09FC,                   //17dbm
  0x09F9,                   //14dbm
  0x09F6,                   //11dbm 
};


__root const u8 SX1276LoRaBwTbl[10] =
{// 0        1     2       3      4       5         6      7       8      9
//7.8KHz,10.4KHz,15.6KHz,20.8KHz,31.2KHz,41.7KHz,62.5KHz,125KHz,250KHz,500KHz
  0,1,2,3,4,5,6,7,8,9
};

__root const u8 SX1276SpreadFactorTbl[7] =
{
  6,7,8,9,10,11,12
};


//__root const u8  SX1276Data[datlen] = {"1234567890ABCDEFGHIJK"};
 u8  SX1276Data[datlen];

u8  gb_RxData[254];                                         //Receive data buffer

void delayms(unsigned int t)
{
	unsigned int i;
	unsigned char j; 
	for(i=0;i<t;i++)
	for(j=0;j<120;j++);
}

void Sx1276M_GpioInt()
{
  //*****RF_rst    PD_ODR_ODR2
  PD_DDR_DDR2=1; //OUTPUT
  PD_CR1_C12=1; //SET PD

 //***** RF_SCK         PC_ODR_ODR4
  PC_DDR_DDR4=1; //SET PD
  PC_CR1_C14=1; //SET PD
  
  
  //*****RF_MISO       PC_IDR_IDR5 //INPUT
  PC_DDR_DDR5=0; //INPUT
  PC_CR1_C15=1; //PULL-UP-INPUT

 //*****RF_MOSI       PC_ODR_ODR6
  PC_DDR_DDR6=1;//OUTPUT
  PC_CR1_C16=1; //SET PD
 
  //*****RF_NSEL_PIN   PC_ODR_ODR7
  PC_DDR_DDR7=1; //OUTPUT
  PC_CR1_C17=1; //SET PD
  

  //*****  RF_IRQ       PD_IDR_IDR3
  PD_DDR_DDR3=0; //INPUT
  PD_CR1_C13=1; //PULL-UP-INPUT
  PD_CR2_C23=0; //

  //***** LED_TX      PB_ODR_ODR0
  PB_DDR_DDR5=1; //OUTPUT 
  PB_CR1_C15=1; //SET PD
  
  //*****   #define LED_RX    PE_ODR_ODR5 //OUTPUT
  PE_DDR_DDR5=1; //OUTPUT
  PE_CR1_C15=1; //SET PD

  //*****     #define SW_CTRL1  		PB_ODR_ODR6
  PB_DDR_DDR6=1; //OUTPUT
  PB_CR1_C16=1; //SET PD

  //*****   #define SW_CTRL2  		PC_ODR_ODR2
  PC_DDR_DDR2=1; //OUTPUT
  PC_CR1_C12=1; //SET PD

  RF_NSEL_PIN=1;
}


void RF_IRQ_DS()
{
PD_CR2_C23=0; //关中断
}
void RF_IRQ_EN()
{
PD_CR2_C23=1; //关中断
}


void SW_Swith_RX()
{
SW_CTRL1=0;
SW_CTRL2=1;
}

void SW_Swith_TX()
{
SW_CTRL1=1;
SW_CTRL2=0;
}
/**********************************************************
**Name:     SPICmd8bit
**Function: SPI Write one byte
**Input:    WrPara
**Output:   none
**note:     use for burst mode
**********************************************************/
void SPICmd8bit(u8 WrPara)
{
  u8 bitcnt;  
  nCS=0;
  SCK=0;
  
  for(bitcnt=8; bitcnt!=0; bitcnt--)
  {
    SCK=0;
    if(WrPara&0x80)
      MOSI=1;
    else
      MOSI=0;
	asm("nop");	
	asm("nop");	
    SCK=1;
	asm("nop");	
    WrPara <<= 1;
  }
  SCK=0;
  MOSI=1;
}

/**********************************************************
**Name:     SPIRead8bit
**Function: SPI Read one byte
**Input:    None
**Output:   result byte
**Note:     use for burst mode
**********************************************************/
u8 SPIRead8bit(void)
{
 u8 RdPara = 0;
 u8 bitcnt;
 
  nCS=0;
  MOSI=1;                                                 //Read one byte data from FIFO, MOSI hold to High
  for(bitcnt=8; bitcnt!=0; bitcnt--)
  {
    SCK=0;
	asm("nop");	
	asm("nop");	
    RdPara <<= 1;
    SCK=1;
    if(MISO)
      RdPara |= 0x01;
    else
      RdPara |= 0x00;
  }
  SCK=0;
  return(RdPara);
}

/**********************************************************
**Name:     SPIRead
**Function: SPI Read CMD
**Input:    adr -> address for read
**Output:   None
**********************************************************/
u8 SPIRead(u8 adr)
{
  u8 tmp; 
  SPICmd8bit(adr);                                         //Send address first
  tmp = SPIRead8bit();  
  nCS=1;
  return(tmp);
}


/**********************************************************
**Name:     SPIWrite
**Function: SPI Write CMD
**Input:    WrPara -> address & data
**Output:   None
**********************************************************/
void SPIWrite(word WrPara)                
{                                                       
  u8 bitcnt;    
  
  SCK=0;
  nCS=0;
  
  WrPara |= 0x8000;                                        //MSB must be "1" for write 
  
  for(bitcnt=16; bitcnt!=0; bitcnt--)
  {
    SCK=0;
	asm("nop");	
	asm("nop");	
    if(WrPara&0x8000)
      MOSI=1;
    else
      MOSI=0;
    SCK=1;
    WrPara <<= 1;
  }
  SCK=0;
  MOSI=1;
  nCS=1;
}

/**********************************************************
**Name:     SPIBurstRead
**Function: SPI burst read mode
**Input:    adr-----address for read
**          ptr-----data buffer point for read
**          length--how many bytes for read
**Output:   None
**********************************************************/
void SPIBurstRead(u8 adr, u8 *ptr, u8 length)
{
  u8 i;
  if(length<=1)                                            //length must more than one
    return;
  else
  {
    SCK=0; 
    nCS=0;
    SPICmd8bit(adr); 
    for(i=0;i<length;i++)
    ptr[i] = SPIRead8bit();
    nCS=1;  
  }
}

/**********************************************************
**Name:     SPIBurstWrite
**Function: SPI burst write mode
**Input:    adr-----address for write
**          ptr-----data buffer point for write
**          length--how many bytes for write
**Output:   none
**********************************************************/
void BurstWrite(u8 adr, u8 *ptr, u8 length)
{ 
  u8 i;

  if(length<=1)                                            //length must more than one
    return;
  else  
  {   
    SCK=0;
    nCS=0;        
    SPICmd8bit(adr|0x80);
    for(i=0;i<length;i++)
    SPICmd8bit(ptr[i]);
    nCS=1;  
  }
}


/**********************************************************
**Name:     SX1276_Standby
**Function: Entry standby mode
**Input:    None
**Output:   None
**********************************************************/
void SX1276_Standby(void)
{
  SPIWrite(LR_RegOpMode+0x01+0x08);                              //Standby
}

/**********************************************************
**Name:     SX1276_Sleep
**Function: Entry sleep mode
**Input:    None
**Output:   None
**********************************************************/
void SX1276_Sleep(void)
{
  SPIWrite(LR_RegOpMode+0x00+0x08);                              //Sleep
}

/*********************************************************/
//LoRa mode
/*********************************************************/
/**********************************************************
**Name:     SX1276_EntryLoRa
**Function: Set RFM69 entry LoRa(LongRange) mode
**Input:    None
**Output:   None
**********************************************************/
void SX1276_EntryLoRa(void)
{
  SPIWrite(LR_RegOpMode+0x80+0x08);
}

/**********************************************************
**Name:     SX1276_LoRaClearIrq
**Function: Clear all irq
**Input:    None
**Output:   None
**********************************************************/
void SX1276_LoRaClearIrq(void)
{
  SPIWrite(LR_RegIrqFlags+0xFF);
}

/**********************************************************
**Name:     SX1276_Config
**Function: SX1276 base config
**Input:    mode
**Output:   None
**********************************************************/
void SX1276_Config(u8 mode)
{
  u8 i; 
    
  RF_RST=0;
  for(i=150;i!=0;i--)                                      //Delay
    asm("NOP"); 
  
  RF_RST=1;
  
  for(i=100;i!=0;i--)                                      //Delay
    asm("NOP");  
    
  SX1276_Sleep();                                           //Change modem mode Must in Sleep mode 
  for(i=100;i!=0;i--)                                      //Delay
    asm("NOP");  

   SPIWrite(REG_LR_TCXO+0x19);                              //USE TCXO

    SX1276_EntryLoRa();  
    //SPIWrite(0x5904);   //?? Change digital regulator form 1.6V to 1.47V: see errata note
    
    for(i=0;i<3;i++)                                       //setting frequency parameter
    {
      SPIWrite(SX1276FreqTbl[i]);  
    }

    //setting base parameter 
    SPIWrite(SX1276PowerTbl[0]);             //Setting output power parameter  
    
    SPIWrite(LR_RegOcp+0x0B);                              //RegOcp,Close Ocp
    SPIWrite(LR_RegLna+0x23);                              //RegLNA,High & LNA Enable

	
    
    if(SX1276SpreadFactorTbl[gb_SF]==6)           //SFactor=6
    {
      u8 tmp;
      SPIWrite(LR_RegModemConfig1+(SX1276LoRaBwTbl[gb_BW]<<4)+(CR<<1)+0x01);//Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
      SPIWrite(LR_RegModemConfig2+(SX1276SpreadFactorTbl[gb_SF]<<4)+(CRC<<2)+0x03);
      
      tmp = SPIRead(0x31);
      tmp &= 0xF8;
      tmp |= 0x05;
      SPIWrite(0x3100+tmp);
      SPIWrite(0x3700+0x0C);
    } 
    else
    {
      SPIWrite(LR_RegModemConfig1+(SX1276LoRaBwTbl[gb_BW]<<4)+(CR<<1)+0x00);//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
      SPIWrite(LR_RegModemConfig2+(SX1276SpreadFactorTbl[gb_SF]<<4)+(CRC<<2)+0x03);  //SFactor &  LNA gain set by the internal AGC loop 
      SPIWrite(LR_RegModemConfig3+0x08);//LowDataRateOptimize en
    }
    SPIWrite(LR_RegSymbTimeoutLsb+0xFF);                   //RegSymbTimeoutLsb Timeout = 0x3FF(Max) 
    
    SPIWrite(LR_RegPreambleMsb + 0);                       //RegPreambleMsb 
    SPIWrite(LR_RegPreambleLsb + 16);                      //RegPreambleLsb 8+4=12byte Preamble  16+4=20
    
    SPIWrite(REG_LR_DIOMAPPING2+0x01);                     //RegDioMapping2 DIO5=00, DIO4=01
    SX1276_Standby();                                         //Entry standby mode
}

/**********************************************************
**Name:     SX1276_LoRaEntryRx
**Function: Entry Rx mode
**Input:    None
**Output:   None
**********************************************************/
void SX1276_LoRaEntryRx(void)
{
  u8 addr; 
        
  SX1276_Config(0);                                         //setting base parameter
  
  SPIWrite(0x4D00+0x84);                                   //Normal and Rx
  SPIWrite(LR_RegHopPeriod+0xFF);                          //RegHopPeriod NO FHSS
  SPIWrite(REG_LR_DIOMAPPING1+0x01);                       //DIO0=00, DIO1=00, DIO2=00, DIO3=01  DIO0=00--RXDONE
      
  SPIWrite(LR_RegIrqFlagsMask+0x3F);                       //Open RxDone interrupt & Timeout
  SX1276_LoRaClearIrq();   
  
  SPIWrite(LR_RegPayloadLength+datlen);                       //RegPayloadLength  21byte(this register must difine when the data long of one byte in SF is 6)
    
  addr = SPIRead((u8)(LR_RegFifoRxBaseAddr>>8));           //Read RxBaseAddr
  SPIWrite(LR_RegFifoAddrPtr+addr);                        //RxBaseAddr -> FiFoAddrPtr　 
  SPIWrite(LR_RegOpMode+0x0D);                        //Continuous Rx Mode

  delayms(10);
}

/**********************************************************
**Name:     SX1276_LoRaRxWaitStable
**Function: Determine whether the state of stable Rx 查询RX 状态
**Input:    none
**Output:   none
**********************************************************/
u8 SX1276_LoRaRxWaitStable(void)
{ 
	uint8 tmp;
	tmp=SPIRead((u8)(LR_RegModemStat>>8));
	return tmp;
}

/**********************************************************
**Name:     SX1276_LoRaRxPacket
**Function: Receive data in LoRa mode
**Input:    None
**Output:   None
**********************************************************/
void SX1276_LoRaRxPacket(void)
{
		u8 addr;
		u8 packet_size;
		Flg_RxFinish=1;
		addr = SPIRead((u8)(LR_RegFifoRxCurrentaddr>>8));//last packet addr 数据包的最后地址(数据的尾地址)
		SPIWrite(LR_RegFifoAddrPtr+addr);//RxBaseAddr -> FiFoAddrPtr   

		if(SX1276SpreadFactorTbl[gb_SF]==6)//When SpreadFactor is six，will used Implicit Header mode(Excluding internal packet length)
			packet_size=21;
		else
			packet_size = SPIRead((u8)(LR_RegRxNbBytes>>8));//Number for received bytes    

		gtmp= packet_size;
		SPIBurstRead(0x00, gb_RxData, packet_size);
		SX1276_LoRaClearIrq();
}


/**********************************************************
**Name:     SX1276_LoRaEntryTx
**Function: Entry Tx mode
**Input:    None
**Output:   None
**********************************************************/
void SX1276_LoRaEntryTx(void)
{
		u8 addr;

		SX1276_Config(0);                                         //setting base parameter

		SPIWrite(0x4D00+0x87);                                   //Tx for 20dBm
		SPIWrite(LR_RegHopPeriod);                               //RegHopPeriod NO FHSS
		SPIWrite(REG_LR_DIOMAPPING1+0x41);                       //DIO0=01, DIO1=00, DIO2=00, DIO3=01

		SX1276_LoRaClearIrq();
		SPIWrite(LR_RegIrqFlagsMask+0xF7);                       //Open TxDone interrupt
		SPIWrite(LR_RegPayloadLength+ datlen);                       //RegPayloadLength  21byte

		addr = SPIRead((u8)(LR_RegFifoTxBaseAddr>>8));           //RegFiFoTxBaseAddr
		SPIWrite(LR_RegFifoAddrPtr+addr);                        //RegFifoAddrPtr
}
uint16 TxPacketCout=1;
/**********************************************************
**Name:     SX1276_LoRaTxPacket
**Function: Send data in LoRa mode
**Input:    None
**Output:   1- Send over
**********************************************************/
void SX1276_LoRaTxPacket(char *data, int len)
{ 
		u8 tmp;
		delayms(10);
		
		SX1276Data[0]=(TxPacketCout>>8);
		SX1276Data[1]=(u8)(TxPacketCout);
		TxPacketCout++;
		
		for(tmp=2;tmp<datlen;tmp++)
			SX1276Data[tmp]=tmp;
		
		BurstWrite(0x00, (u8 *)SX1276Data, datlen);
		SPIWrite(LR_RegOpMode+0x03+0x08);                    //Tx Mode       
}



///////////////////////////////////////////////// FSK mode //////////////////////////////////////////////////
const u16 SX1276ConfigTbl[16] = 
{ 
  0x0402,                   //RegFdevMsb 	35KHz 
  0x053D,                   //RegFdevLsb
  0x0B0B,                   //RegOcp 	Close Ocp
  //0x0C20,                 //RegLNA 	High & LNA Disable
  0x0C23,                   //RegLNA	High & LNA Enable
  0x1212,                   //RegRxBw		83KHz
  0x1FA0,                   //RegPreambleDet	Enable 2Byte 
  //0x1F20,                 //RegPreambleDet	Disable 
  0x2500,                   //RegPreambleMsb  
  0x2606,                   //RegPreambleLsb	6Byte Preamble
  0x2792,                   //RegSyncConfig	Sync 2+1=3bytes
  0x2800+0xAA,              //SyncWord = aa2dd4
  0x2900+0x2D,              //
  0x2A00+0xD4,              //
  0x3000,                   //RegPacketConfig1  Disable CRC，NRZ
  0x3140,                   //RegPacketConfig2  Packet Mode
  0x3215,                   //RegPayloadLength  21bytes Fixed
  0x3595,                   //RegFiFoThresh		21bytes                        
};

const u16 SX1276FSKRateTbl[4][2] = 
{
  {0x0268, 0x032B},         //BR=1.2Kbps
  {0x0234, 0x0315},         //BR=2.4Kbps
  {0x021A, 0x030B},         //BR=4.8Kbps
  {0x020D, 0x0305},         //BR=9.6Kbps
};

const u16 SX1276RxTable[4] = 
{       
  0x090F,                   //RFIO Pin
  0x400C,                   //DIO0 Mapping for IRQ / DIO2 for RxData
  0x4100,                   //
  0x4D84,                   //Normal and Rx   
};
                              
const u16 SX1276TxTable[3] = 
{
  0x4000,                   //DIO0 Mapping for IRQ / DIO2 for RxData
  0x4100,                   //          
  0x4D87,                   //20dBm Tx
};
/**********************************************************
**Name:     SX1276_Config
**Function: SX1276 base config
**Input:    mode
**Output:   None
**********************************************************/
void SX1276_FskConfig()
{
  u8 i; 

 RF_RST=0;
  for(i=100;i!=0;i--)                                      //Delay
    asm("NOP"); 
  
  RF_RST=1;
  
  for(i=250;i!=0;i--)                                      //Delay
    asm("NOP");  
    
  SX1276_Sleep();                                           //Change modem mode Must in Sleep mode 
  for(i=250;i!=0;i--)                                      //Delay
    asm("NOP");  
  
    for(i=0;i<3;i++)                                       //setting frequency parameter
    {
      SPIWrite(SX1276FreqTbl[i]);  
    }
	
    SPIWrite(SX1276PowerTbl[0]);             //Setting output power parameter
    
    for(i=0;i<16;i++)                                      //setting base parameter
      SPIWrite(SX1276ConfigTbl[i]);

  SX1276_Standby();                                         //Entry standby mode
}

/**********************************************************
**Name:     SX1276_FskClearFIFO
**Function: Change to RxMode from StandbyMode, can clear FIFO buffer
**Input:    None
**Output:   None
**********************************************************/
void SX1276_FskClearFIFO(void)
{
  SPIWrite(0x0101);                                        //Standby
  SPIWrite(0x0105+0x08);                                   //entry RxMode
}

/**********************************************************
**Name:     SX1276_FskEntryRx
**Function: Set RFM69 entry FSK Rx_mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
void SX1276_FskEntryRx(void)
{
  u8 i;
  
  SX1276_FskConfig();  
  for(i=0;i<2;i++)
    SPIWrite(SX1276FSKRateTbl[0][i]);         //setting rf rate parameter  1.2kbps
  for(i=0;i<4;i++)                                         //Define to Rx mode 
    SPIWrite(SX1276RxTable[i]);    
  SPIWrite(0x0105+0x08);                                   //entry RxMode
  
}


/**********************************************************
**Name:     SX1276_FskRxPacket
**Function: Check for receive one packet
**Input:    none
**Output:   "!0"-----Receive one packet
**          "0"------Nothing for receive
**********************************************************/
u8 SX1276_FskRxPacket(void)
{
  u8 i; 
    
  if(RF_IRQ_DIO0)
  { 
    for(i=0;i<32;i++) 
      gb_RxData[i] = 0x00;  
    
    SPIBurstRead(0x00, gb_RxData, 21);  
    SX1276_FskClearFIFO();
    for(i=0;i<17;i++)
    {
      if(gb_RxData[i]!=SX1276Data[i])
        break;  
    }
    if(i>=17) 
      return(1);                                           //Rx success
    else
      return(0);
  }
  else
    return(0);  
}



/**********************************************************
**Name:     SX1276_FskEntryTx
**Function: Set SX1276 entry FSK Tx_mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
void SX1276_FskEntryTx(void)
{
  u8 i;
  
  SX1276_FskConfig();
  for(i=0;i<2;i++)
    SPIWrite(SX1276FSKRateTbl[0][i]);         //setting rf rate parameter  
  for(i=0;i<3;i++)                                         //Define to Tx mode 
    SPIWrite(SX1276TxTable[i]);
  SPIWrite(0x0103+0x08);
      
}

/**********************************************************
**Name:     SX1276_FskTxPacket
**Function: Check SX1276 send over & send next packet
**Input:    none
**Output:   TxFlag=1, Send success
**********************************************************/
void SX1276_FskTxPacket(void)
{
		
		BurstWrite(0x00, (u8 *)SX1276Data, 21);     
		SPIWrite(0x0103+0x08);                               //Entry Tx mode

		asm("nop");	

		while(!RF_IRQ_DIO0) ;                    //Packet send over 发送完成了IRQ 变为H,平时L

		SX1276_Standby();                                     //Entry Standby mode      

  
}








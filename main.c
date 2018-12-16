
#include  <iostm8s103f3.h>
#include "My_type.h"
#include "main.h"
#include "sx1276-LoRa.h"

 void UART1_init(void)
{
	UART1_CR2=0;
	UART1_SR=0;
	UART1_CR1=0;
	UART1_CR3=0;
	UART1_BRR2 = 0x00;         // ���ò�����19200
	UART1_BRR1 = 0x1A;         // 8M/2400 = 0x1A0
	UART1_CR2=0x00;//
}
void TIM1_init(void)
{
  TIM1_PSCRH = 0x1F;  // 8Mϵͳʱ�Ӿ�Ԥ��Ƶf=fck/(PSCR+1)
  TIM1_PSCRL = 0x3F;  // PSCR=0x1F3F��f=8M/(0x1F3F+1)=1000Hz��ÿ����������1ms
  TIM1_ARRH = 0x00;  // �Զ����ؼĴ���ARR=0x01F4=500
  TIM1_ARRL = 0x64;  // ÿ����100�β���һ���жϣ���100ms
  TIM1_IER = 0x01;    // ��������ж�
  TIM1_CR1 = 0x01;  // ������ʹ�ܣ���ʼ����
}

void EXTI_init(void)
{
  EXTI_CR1 = 0x40; // PD�������ش����ж� //ƽʱDIO0 ��L���յ�һ�����ݲ���CRC��ȷ��DIO0 ��ΪH
}

void Sx1276EV_IO_Inital()
{	
	//S_Tx_Rx
	PC_DDR_DDR1=0;//S_Tx_Rx
	PC_CR1_C11=1; ////PULL-UP-INPUT
	PC_CR2_C21=0; //
	//S_BW10
	PA_DDR_DDR1=0;//
	PA_CR1_C11=1; ////PULL-UP-INPUT
	PA_CR2_C21=0; 

	//S_BW62
	PA_DDR_DDR2=0;//
	PA_CR1_C12=1; ////PULL-UP-INPUT
	PA_CR2_C22=0;

	//S_BW125
	PB_DDR_DDR3=0;//
	PB_CR1_C13=1; ////PULL-UP-INPUT
	PB_CR2_C23=0; 

	//S_FSK
	PB_DDR_DDR7=0;//
	PB_CR1_C17=1; ////PULL-UP-INPUT
	PB_CR2_C27=0; 
}

void KeyRead( void )
{
	u8 ReadData;
	/*Flg_S_BW10=S_BW10;//
	Flg_S_BW62=S_BW62;//
	Flg_S_BW125=S_BW125;//
	Flg_S_FSK=S_FSK;//
        */
	Flg_S_Tx_Rx=S_Tx_Rx;
	ReadData=Flg^0xff;
    Trg = ReadData & (ReadData ^ Cont);     
    Cont = ReadData;                           
    Cont &= 0x1F;
	if((Cont!=ContBak)&&(ContBak!=0xff))
		WWDG_CR=0X8F;//STM8 �����λ����ϵͳ
        
}

void SX1276_Parameters_Select()
{
			ContBak=Cont;
			if((Cont&0x10)==cTx_Statue	|| true)//MOCK
				{
					Flg_SX1276_RXorTX=Sx1276_TxMode;
					Tx_phase=SetTx_Parameters;
				}
			else
				{
					Flg_SX1276_RXorTX=Sx1276_RxMode;
					Rx_phase=Rx_Parameters_Set;
				}
		
			switch(Cont&0x0F)
				{
					case 1:
						/****BW ѡ��*********/
						gb_BW=1;//10.4K  BW
						//gb_BW=2;//15.6K  BW 
						//gb_BW=3;//20.8K  BW 
						//gb_BW=4;//31.2K  BW 
						//gb_BW=5;//41.7K  BW 
						//gb_BW=6;//62.5K  BW 
						//gb_BW=7;//125KHz BW 
						/****BW ѡ�����*********/
						
						/*******SF ѡ��*********/
						//gb_SF=0;// SF=6;  64 chips / symbol
						//gb_SF=1;// SF=7;  128 chips / symbol
						//gb_SF=2;// SF=8;  256 chips / symbol
						//gb_SF=3;// SF=9;  9  512 chips / symbol
						gb_SF=4;////SF=10; 1024 chips / symbol
						//gb_SF=5;////SF=11; 2048 chips / symbol
						//gb_SF=6;//SF=12;  4096 chips / symbol
						/****SF ѡ�����*********/
						
						/*******CR ѡ��*********/
						//CR=1;////  Error coding rate=4/5
						CR=2;////  Error coding rate=4/6
						//CR=3;////  Error coding rate=4/7
						//CR=4;////  Error coding rate=4/8
						/*******CR ѡ�����*********/
						break;

					case 2:
						/****BW ѡ��*********/
						//gb_BW=1;//10.4K  BW
						//gb_BW=2;//15.6K  BW 
						//gb_BW=3;//20.8K  BW 
						//gb_BW=4;//31.2K  BW 
						//gb_BW=5;//41.7K  BW 
						gb_BW=6;//62.5K  BW 
						//gb_BW=7;//125KHz BW 
						/****BW ѡ�����*********/
						
						//gb_SF=0;// SF=6;  64 chips / symbol
						//gb_SF=1;// SF=7;  128 chips / symbol
						//gb_SF=2;// SF=8;  256 chips / symbol
						//gb_SF=3;// SF=9;  9  512 chips / symbol
						//gb_SF=4;////SF=10; 1024 chips / symbol
						gb_SF=5;////SF=11; 2048 chips / symbol
						//gb_SF=6;//SF=12;  4096 chips / symbol
						
						/*******CR ѡ��*********/
						//CR=1;////  Error coding rate=4/5
						CR=2;////  Error coding rate=4/6
						//CR=3;////  Error coding rate=4/7
						//CR=4;////  Error coding rate=4/8
						/*******CR ѡ�����*********/
						break;
					
					case 4:  //125KHz BW   SF=10;  Error coding rate=4/6
						/****BW ѡ��*********/
						//gb_BW=1;//10.4K  BW
						//gb_BW=2;//15.6K  BW 
						//gb_BW=3;//20.8K  BW 
						//gb_BW=4;//31.2K  BW 
						//gb_BW=5;//41.7K  BW 
						//gb_BW=6;//62.5K  BW 
						gb_BW=7;//125KHz BW 
						/****BW ѡ�����*********/

						/*******SF ѡ��*********/
						//gb_SF=0;// SF=6;  64 chips / symbol
						//gb_SF=1;// SF=7;  128 chips / symbol
						//gb_SF=2;// SF=8;  256 chips / symbol
						//gb_SF=3;// SF=9;  9  512 chips / symbol
						gb_SF=4;////SF=10; 1024 chips / symbol
						//gb_SF=5;////SF=11; 2048 chips / symbol
						//gb_SF=6;//SF=12;  4096 chips / symbol
						/****SF ѡ�����*********/

						/*******CR ѡ��*********/
						//CR=1;////  Error coding rate=4/5
						CR=2;////  Error coding rate=4/6
						//CR=3;////  Error coding rate=4/7
						//CR=4;////  Error coding rate=4/8
						/*******CR ѡ�����*********/
						break;
					case 8:
						//FSK
						//break; //MOCK
					default://10.4K   SF=7; cr=4/6
						/****BW ѡ��*********/
						gb_BW=1;//10.4K  BW
						//gb_BW=2;//15.6K  BW 
						//gb_BW=3;//20.8K  BW 
						//gb_BW=4;//31.2K  BW 
						//gb_BW=5;//41.7K  BW 
						//gb_BW=6;//62.5K  BW 
						//gb_BW=7;//125KHz BW 
						/****BW ѡ�����*********/

						/*******SF ѡ��*********/
						//gb_SF=0;// SF=6;  64 chips / symbol
						gb_SF=1;// SF=7;  128 chips / symbol
						//gb_SF=2;// SF=8;  256 chips / symbol
						//gb_SF=3;// SF=9;  9  512 chips / symbol
						//gb_SF=4;////SF=10; 1024 chips / symbol
						//gb_SF=5;////SF=11; 2048 chips / symbol
						//gb_SF=6;//SF=12;  4096 chips / symbol
						/****SF ѡ�����*********/

						/*******CR ѡ��*********/
						//CR=1;////  Error coding rate=4/5
						CR=2;////  Error coding rate=4/6
						//CR=3;////  Error coding rate=4/7
						//CR=4;////  Error coding rate=4/8
						/*******CR ѡ�����*********/
						break;	
				}
	}


/***********************************************
�� ��:	send_char_com( unsigned char ch)
�� ��:	���ڷ���һ��ʮ����������PC
*********************************************** */  
void send_char_com(u8 UtxData)
{
	UART1_CR2_TEN=1;
 	while(!UART1_SR_TXE);
  		UART1_DR = UtxData;         //
   	while(!UART1_SR_TC);//TC==0 ������� �ȴ�UART ����������ɣ�
		UART1_CR2_TEN=0;
} 


void Uart_Prints(uint8 * pd)
{
	while((*pd)!='\0')
	{
	send_char_com(*pd);
	pd++;
	}
}

/**********************************************************
//����1λС����i��Ӧ��<2��   2λС����i<4;   3λС����i<5��
//dividend-����������ĸ��  divisor-���������ӣ�                   
**********************************************************/ 
void Float_Division(uint16 dividend,uint16 divisor )
{
	uint8 i;
	for(i = 0; i < 5;i++)
	{
  		D_value[i] = dividend/divisor;
  		dividend = dividend%divisor;
  		if(dividend > 0)
  			{
   				dividend = dividend *10;
  			} 
	}
}

void HexToAscii_AndUartSent(uint8 Hex)
{
uint8 Ascii[3];
Ascii[0]=(Hex/100)+0x30;
Ascii[1]=((Hex%100)/10)+0x30;
Ascii[2]=(Hex%10)+0x30;
send_char_com(Ascii[0]);
send_char_com(Ascii[1]);
send_char_com(Ascii[2]);
}

void PER_Proc()
{
	uint8 p_total;
	uint8 p_lost;
	switch(PER_phase)//SetTx_Parameters
	       			{
	       				case PerStart:
								RxPacketCout=1;
								
								PacketNuStart.U8[MSB]=gb_RxData[0];
								PacketNuStart.U8[LSB]=gb_RxData[1];
								
								PER_phase=PerGoOn;//payload
								Uart_Prints((u8 *)cPerStart);
								break;
						case PerGoOn:
								RxPacketCout++;
								
								PacketNuNow.U8[MSB]=gb_RxData[0];//MSB
								PacketNuNow.U8[LSB]=gb_RxData[1];

								Uart_Prints((u8 *)cGET);
								HexToAscii_AndUartSent(RxPacketCout);
								
								Uart_Prints((u8 *)cToatl);
								p_total=(PacketNuNow.U16-PacketNuStart.U16+1);
								HexToAscii_AndUartSent(p_total);
								
								Uart_Prints((u8 *)cLost);
								p_lost=p_total-RxPacketCout;
								HexToAscii_AndUartSent(p_lost);
								
								Uart_Prints((u8 *)cPer);							
								Float_Division(p_lost,p_total);///ֵ����D_value[5]���棬D_value[0] ���λ //PER=(p_lost/p_total);
								send_char_com(D_value[0]+0x30);
								send_char_com(0x2e);//С����
								send_char_com(D_value[1]+0x30);
								send_char_com(D_value[2]+0x30);
								send_char_com(D_value[3]+0x30);
								send_char_com(D_value[4]+0x30);
								
								send_char_com(0x0D);//����
								send_char_com(0x0A);//����

								if(PacketNuNow.U16<PacketNuStart.U16)
									PER_phase=PerStart;
                                                                
								if((PacketNuNow.U16-PacketNuStart.U16)>=99)
									PER_phase=PerStart;
									
								break;
							
						case PerDone:
								break;
					}

}

void main( void )
{ 
    CLK_CKDIVR = 0x08;         // 16M�ڲ�RC��2��Ƶ��ϵͳʱ��Ϊ8M
    char sendBuf[256] = {'0'};
    int bufLen = 256;
	asm("sim");  				// ��ȫ���ж�
	Sx1276EV_IO_Inital();	//EVB IO ��ʼ��
    TIM1_init();
	EXTI_init();
	UART1_init();

	Sx1276M_GpioInt();//SX1276 IO��ʼ��
	RF_RST=1;
	Sx1276VerNO = SPIRead((u8)(REG_LR_VERSION>>8));//��ȡSX1276 �汾�� 0X11(V1A�汾 ���̰棩����0X12��V1B ��ʽ�棩
	
	LED_RX=1;
	LED_TX=1;
	Flg_RxFinish=0;
	KeyRead();
    SX1276_Parameters_Select();
    asm("rim");  // ��ȫ���ж�
	while(1)
	{   
          //firsr we make send side
          SW_Swith_TX();
          SX1276_LoRaEntryTx();
          //Tx_phase=Write_FIFO_AndTx;
          LED_TX=0;

          SX1276_LoRaTxPacket(sendBuf,bufLen);// send 
          LED_TX=1;
 
          //Tx_phase=Wait_Tx_Finish;
		/*if(Flg_SX1276_RXorTX==Sx1276_TxMode)//����ģʽ,�����Ƿ�������
			{
				switch(Tx_phase)//SetTx_Parameters
	       			{
	       				case SetTx_Parameters:
								SW_Swith_TX();
								SX1276_LoRaEntryTx();
								Tx_phase=Write_FIFO_AndTx;
								break;
									
						case Write_FIFO_AndTx:
								SX1276_LoRaTxPacket();
								Tx_phase=Wait_Tx_Finish;
								LED_TX=0;
								break;
								
						case Wait_Tx_Finish:
								if(RF_IRQ_DIO0)                     //Packet send over ���������IRQ ��ΪH,ƽʱL
									{
										SX1276_LoRaClearIrq(); //Clear irq
										SX1276_Standby(); //Entry Standby mode  
										TxTimeCout=0;
										Tx_phase=Wait_NextTx;
										LED_TX=1;
									}
								break;
						case Wait_NextTx:
								if(TxTimeCout>9)///1s ������
									{
										Tx_phase=SetTx_Parameters;
										TxTimeCout=0;
									}
								break;
	       			}
			}
		
		else //����ģʽ
			{
			  	if((SX1276_GetStatue_Cout>100)&&(Flg_RxFinish==0))//��ʱ��ȡSX1276 ��״̬��ȷ��SX1276 ��RX״̬
			  		{
			  			SX1276_GetStatue_Cout=0;
						Sx1276Statue = SPIRead((u8)(LR_RegOpMode>>8));
							 if(Sx1276Statue!=cSx1276RxStatue)
								Rx_phase=Rx_Parameters_Set;
			  		}
				 switch(Rx_phase)//
	       			{
	       				case Rx_Parameters_Set:
								SW_Swith_RX();
								SX1276_LoRaEntryRx();
								RF_IRQ_EN();
								SX1276_GetStatue_Cout=150;
								Rx_phase=Wait_RxFinish;
								break;
						case Wait_RxFinish:
								if(Flg_RxFinish)
									{
										Flg_RxFinish=0;
										LED_RX=~LED_RX;
										PER_Proc();//PER ������򣬴�UART���(������19200bps)PER ��Ϣ��100����Ϊһ��Cycle��
									}
								break;
	       			}
			}*/
	}
}


/**********************************************************
	�ⲿ�ж�PE���                    
**********************************************************/ 
#pragma vector = 8     
__interrupt void EXTI_PD(void)
{
	SX1276_LoRaRxPacket();
}

/**********************************************************
	Timer1 �ж����                    
**********************************************************/ 
#pragma vector=TIM1_OVR_UIF_vector
__interrupt void TIM1_OVR_UIF(void)//��ʱ80ms һ������
{
	SX1276_GetStatue_Cout++;
	TxTimeCout++;
	KeyRead();//����ɨ��
  	TIM1_SR1 = 0;  // ��������жϱ��
}
/**********************************************************
	�������           
**********************************************************/ 


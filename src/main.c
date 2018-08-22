/* upon power on, we look for a udp server for 60 seconds-if we don't find one we use our default ip, port and gateway address
 * next we send a broadcast message on the subnet to allow computers to see our ip address
 * the computer then sends the ! packet to which we respond with the # packet.
 * this requires the computer to see the broadcast packet
 */
#include <stddef.h>
#include <string.h>
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

#define u8 uint8_t
#define u8_t uint8_t
#define u16 uint16_t
#define u16_t uint16_t
#define u32 uint32_t

#define NEWBOARD //moves the serial port to portA pins 2 and 3
//#define DEBUD //prints a bunch of stuff out during DHCP connection process
#define ETHER //if defined we initialize the Ethernet and send the IR out the ethernet port rather than the serial port

#define CMR 0,0 //the common register block mode register
#define CPHY 0x2e,0 //we don't ever write to this register
#define CVERSION 0x30,0

#define S0MR 0,1
#define S0CR 1,1
#define S0IR 2,1
#define S0SR 3,1 //the socket status register-in udp mode only two values are used 0x00(socket closed) and 0x22(socket udp)
#define S0CONFIG 4,1 //we write 15 bytes-first byte is S0 source port

#define S0TXSIZ 0x1f,1
#define S0TXFSR 0x20,1
#define S0TXRPTR 0x22,1
#define S0TXWPTR 0x24,1

#define S0RXRCV 0x26,1
#define S0RXRPTR 0x28,1
#define S0RXWPTR 0x2a,1

#define S0TX 0,2
#define S0RX 0,3

#define LEDon GPIOA->BSRR=GPIO_Pin_1
#define LEDoff GPIOA->BRR=GPIO_Pin_1

#define INDUCTIVEon GPIOA->BSRR=GPIO_Pin_0
#define INDUCTIVEoff GPIOA->BRR=GPIO_Pin_0

#define EECShi GPIOB->BSRR=GPIO_Pin_6
#define EECSlo GPIOB->BRR=GPIO_Pin_6

#define WCShi GPIOB->BSRR=GPIO_Pin_7
#define WCSlo GPIOB->BRR=GPIO_Pin_7

#define WRSThi GPIOB->BSRR=GPIO_Pin_0
#define WRSTlo GPIOB->BRR=GPIO_Pin_0

#define REV0 '0'
#define REV1 '0'
#define REV2 '0'
#define REV3 '1'//can be a-z

#define IRSPEED 4800//14400

#define DHCP_REQUEST        1
#define DHCP_REPLY          2
#define DHCP_HTYPE_ETHERNET 1
#define DHCP_HLEN_ETHERNET  6

#define DHCP_SERVER_PORT  67
#define DHCP_CLIENT_PORT  68
#define DHCP_OPTION_END         255

#define DHCPDISCOVER  1
#define DHCPOFFER     2
#define DHCPREQUEST   3
#define DHCPDECLINE   4
#define DHCPACK       5
#define DHCPNAK       6
#define DHCPRELEASE   7

struct dhcp_state {
  struct uip_udp_conn *conn;
  const void *mac_addr;
  u8 mac_len;
  u8_t serverid[4];
  u16_t lease_time[2];
  u16_t ipaddr[2];
  u16_t netmask[2];
  u16_t dnsaddr[2];
  u16_t default_router[2];
};

struct dhcp_msg {//the total length is 12+32+64+128+312=548
  u8_t op, htype, hlen, hops;//4 bytes
  u8_t xid[4];//4 bytes
  u16_t secs, flags;//4 bytes
  u8_t ciaddr[4];
  u8_t yiaddr[4];
  u8_t siaddr[4];
  u8_t giaddr[4];
  u8_t chaddr[16];//client hardware address
  u8_t sname[64];
  u8_t file[128];
  u8_t options[312];
};

//GLOBAL VARIABLES
const char CRLF[]="\n\r";
const char PROMPT[]="\n\r>";
const char HELLO[]="Twitch_Dock_";
const char ETHERSTR[]="ETHER_";
const char FAILED[]="FAILED ";

const char TCPOPEN[]="TCP_OPEN\n\r";
const char TCPCONNECTION[]="TCP_Connection\n\r";
const char MQTTCONNECTION[]="MQTT_Connection\n\r";
const char MQTTEXPECTED[]="MQTT_Response_4_Expected Received:";
const char MQTTACCEPTED[]="MQTT Accepted\n\r";
const char MQTTRESPONSE[]="MQTT Response:";

const u8 MAC[]={0,8,0xDC,1,2,4};
const u8 PUBLISH[]={0x30,12,0,8,'5','2','8','/','T','e','m','p','6','0'};
const u8 CONNECT31[]={0x10,18,0,6,'M','Q','I','s','d','p',3,2,0,60,0,4,'d','o','u','g'};
#define CONNECT31LEN 20
const u8 CONNECT311[]={0x10,16,0,4,'M','Q','T','T',4,2,0,60,0,4,'d','o','u','g'};
#define CONNECT311LEN 18

#define GDLEN 240  //our diagnostic output buffer-note that we must manually check for overflow when adding to this buffer-should we ifdef this?
char gdbuff[GDLEN]; //the circulating diagnostic output buffer
u8 gdcnt;//this is a count of how many characters are in the output buffer

#define GBUFFLEN 250 //this 250 byte buffer can be used for anything
u8 gbuff[GBUFFLEN];

#define GIRINLEN 17
u8 girinbuff[GIRINLEN];//this is the receive buffer for the ir port-we receive 17 byte packets
u8 girincnt;

#define GIROUTLEN 10 //we could send a ! followed by 4 bytes this would potentially allow 2 uplink packets in the buffer
u32 giroutbuff[GIROUTLEN];
u8 giroutcnt;

u8 gtimer;

u8 gstate; //goes from 0 to 1 when we are connected to MQTT

u8 gtesttof=100;//this is just for our test packets

u16 girpktcnt;

u16 gseconds;

//FUNCTION DECLARATIONS
void SysTick_Handler(void);

void InitIO(void);
void InitSPI1(void);
void InitUsart2(void);
void InitUsart1(void);//the ir port

void InitCMR(void); //inits the 1st 19 bytes of the common area
void InitSOMR(void);//inits socket 0
void InitS0CONFIG(void);
void InitS0CONFIGBC(void);
void InitS0CR(u8 x);

u16 MQTTconnect(u8 x);

void s25ferall(void);//erase the entire eeprom
u8 s25fstat(u8 x);
u16 s25fid(void);
void s25frd(u16 blk);//with the 32mb device blocks go 0-7ff
u16 s25fnumnotclr(void); //returns the number of blocks that are not empty

void wrc(u16 offset,u8 bsb,u8* p,u8 cnt);//reads n bytes from the bsb (0-common) (1-sock0) (2-skt0rx) (3-sck0tx)
void wwc(u16 offset,u8 bsb,u8* p,u8 cnt);//writes n bytes to the bsb at offset
void wreadpkt(u8* buff, u16 rptr,u16 len);//read len bytes into rptr
void wsendpkt(u8* buff,u8 len); //sends a packet of length len pointed to by buff

void xb2(u8 x,int8_t* buff,u8 format);
void xb3(u16 x,int8_t* buff);
void xb4(u16 x,int8_t* buff,u8 spaceleadingzero);
void xb5(u16 x,u8* buff);
u8 xb5s(u16 x,int8_t* buff);
void xd2(u8 x);
void xd3(u8 x);
void xd5(u16 x);
void xdc(char x);
u8   xdf(const char* pptr);
void xdp(char c);//prints a paren around a character
void xdshort(short x);
void xh(u8 x);
void xh2(u8 x);
void xh4(u16 x);
void xh8(u32 x);
void xques();

void xirc(char x);//sends a character up the ir link
void xirack();//increments girpktcnt and puts the 5 character acknowledgement on the output queue

//FUNCTIONS
void SysTick_Handler(void){
  gtimer=1;
}
void MQTTserialize(const u8* p,u8 len){//serialize the message into gbuffer
	u8 i;
	for(i=0;i<len;i++)gbuff[i]=p[i];
}
u16 MQTTconnect(u8 x){//x is either 3 for MQTT 3.1 or 4 for MQTT 3.1,1 returns teh number of bytes sent
	u16 k=x==3?CONNECT31LEN:CONNECT311LEN;
	MQTTserialize(x==3?CONNECT31:CONNECT311,k);
	wsendpkt(gbuff,k);//sends the pkt including waiting until complete
	return k;
}
u16 MQTTpublish(void){
	MQTTserialize(PUBLISH,14);
	gbuff[13]=(gseconds%10) +0x30;
	wsendpkt(gbuff,14);//sends the pkt including waiting until complete
	return 14;
}
u8 MQTTaccept(void){//returns 0 if the MQTT connect was accepted 1 if not
	return 0;
}
void wreadpkt(u8* buff, u16 ptr,u16 len){//returns the number of bytes read
	wrc(S0RX,gbuff,len);//socket 0 number of bytes read
}
void wsendpkt(u8* p,u8 len){
	u8 buff[2],done=0;
	u16 k,m;

	wrc(S0TXWPTR,buff,2); 	//get the current tx write pointer
	k=(u16)(buff[0])<<8;
	k+=buff[1];
	wwc(k,2,p,len);			//put the data in the transmit buffer
	k+=len;
	buff[0]=(u8)(k>>8);
	buff[1]=(u8)(k&0xff);
	wwc(S0TXWPTR,buff,2);   //update the write pointer
	buff[0]=0x20;
	wwc(S0CR,buff,1);		//send it out
	//now we wait for the operation to complete
	while(!done){
		wrc(S0TXRPTR,buff,2);
		m=(u16)(buff[0])<<8;
		m+=buff[1];
		if(k==m)done=1;
	}
}

void InitS0CR(u8 x){//send x to the S0 command register
	u8 buff[1];
	buff[0]=x;
	wwc(S0CR,buff,1);
}
void InitS0CONFIG(void){
	u8 i,buff[18];
	buff[0]=6;//source port
	buff[1]=0xa7;
	for(i=2;i<8;i++)buff[i]=0;//this is the destination hardware address-it is set as a result of arp
	buff[8]=198; 	//37;//198;//destination ip address
	buff[9]=57; 	//187;//57;
	buff[10]=47;	//106;//47;
	buff[11]=236;	//16;//236;
	buff[12]=0x7;//port 1883
	buff[13]=0x5b;
	buff[14]=5;
	buff[15]=0xb4;//sets the maximum packet size to 1460
	buff[16]=0;//type of service field
	buff[17]=8;//time to live field
	wwc(S0CONFIG,buff,18);
}
void InitS0CONFIGBC(void){
	u8 i,buff[18];
	buff[0]=6;//source port
	buff[1]=0xa7;
	for(i=2;i<8;i++)buff[i]=0;
	buff[8]=0xff;//destination ip address
	buff[9]=0xff;
	buff[10]=0xff;
	buff[11]=0xff;//we send from 34(me) to 35
	buff[12]=0x6;
	buff[13]=0xa7;
	buff[14]=5;
	buff[15]=0xa4;//sets the maximum packet size to 0x5a4
	buff[16]=0;//this is the type of serrvice field
	buff[17]=8;//this is the time to live field
	wwc(S0CONFIG,buff,18);
}
void InitS0MR(void){
	u8 buff[1];
	buff[0]=0x1; //1 is tcp 2 is udp
	wwc(S0MR,buff,1);
}
void InitCMR(void){//this is my configuration
	u8 i,buff[19];
	buff[0]=0x80;//reset
	buff[1]=192;//gateway
	buff[2]=168;
	buff[3]=1;
	buff[4]=1;
	buff[5]=255;//subnet mask
	buff[6]=255;
	buff[7]=255;
	buff[8]=0;
	for(i=0;i<6;i++)buff[i+9]=MAC[i];//called source hardware address
	buff[15]=192;//source ip address
	buff[16]=168;
	buff[17]=1;
	buff[18]=34;
	wwc(CMR,buff,19);
}
void xirack(void){//sends out the 5 byte acknowledgement-!followed by 4 hex digits which give the count 0-0xffff
	u8 i;

	xirc('!');

	girpktcnt++;
	i=(u8)(girpktcnt>>12);//0-0xf
	i+=i<10?0x30:55;
	xirc(i);

	i=(u8)(girpktcnt>>8)&0xf;//0-0xf
	i+=i<10?0x30:55;
	xirc(i);

	i=(u8)(girpktcnt>>4)&0xf;//0-0xf
	i+=i<10?0x30:55;
	xirc(i);

	i=(u8)(girpktcnt&0xf);
	i+=i<10?0x30:55;
	xirc(i);
}
void wwc(u16 offset,u8 bsb,u8*p,u8 cnt){
	uint32_t spixbase=(uint32_t)(SPI1)+0x0c;
	u8 i;

	WCSlo;
	*(__IO uint8_t *) spixbase = offset>>8;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	*(__IO uint8_t *) spixbase = offset&0xff;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	*(__IO uint8_t *) spixbase = (bsb<<3)+4;//selects write the bsb in variable length mode
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	for(i=0;i<cnt;i++){
		*(__IO uint8_t *) spixbase = p[i];
		while (SPI1->SR&SPI_SR_BSY);//wait until not busy
		*(__IO uint8_t *) spixbase;//pick it up
	}
	WCShi;
}

void wrc(u16 offset,u8 bsb,u8*p,u8 cnt){//the bsb is 5 bits long and has to be left shifted 3
	uint32_t spixbase=(uint32_t)(SPI1)+0x0c;
	u8 i;

	WCSlo;
	*(__IO uint8_t *) spixbase = (u8)(offset>>8);
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	*(__IO uint8_t *) spixbase = (u8)(offset&0xff);
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	*(__IO uint8_t *) spixbase = bsb<<3;//selects read from the bsb-the bottom 3 bits are zero
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	for(i=0;i<cnt;i++){
		*(__IO uint8_t *) spixbase = 0;
		while (SPI1->SR&SPI_SR_BSY);//wait until not busy
		p[i]=*(__IO uint8_t *) spixbase;//pick it up
	}
	WCShi;
}

void s25ferall(void){
	uint32_t spixbase=(uint32_t)(SPI1)+0x0c;

	EECSlo;
	*(__IO uint8_t *) spixbase = 6;//6 is wren command
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	EECShi;

	EECSlo;
	*(__IO uint8_t *) spixbase = 0xc7;//erase the entire chip
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	EECShi;//we have to check the status
}

//x is 0x5,0x35,0x33 depending on which status register we want read
u8 s25fstat(u8 x){
	uint32_t spixbase=(uint32_t)(SPI1)+0x0c;
	u8 i;

	EECSlo;
	*(__IO uint8_t *) spixbase = x;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	*(__IO uint8_t *) spixbase = 0;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	i=*(__IO uint8_t *) spixbase;//pick it up
	EECShi;
	return i;
}
//32megabit device 4megabytes 256 byte blocks-highest block 0x3fff
void s25frd(u16 blk){
	u8* p=(u8*)gbuff;
	u16 j;
	uint32_t spixbase=(uint32_t)(SPI1)+0x0c;

	EECSlo;
	*(__IO uint8_t *) spixbase = 3;//3 is the read command
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	*(__IO uint8_t *) spixbase = (u8)(blk>>8);
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	*(__IO uint8_t *) spixbase = (u8)(blk&0xff);
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	*(__IO uint8_t *) spixbase = 0;//the lowest byte of the addr is always 0
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	for(j=0;j<256;j++){
	*(__IO uint8_t *) spixbase = 0;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	p[j]=*(__IO uint8_t *) spixbase;//pick it up
	}
	EECShi;
}
u16 s25fid(void){
	uint32_t spixbase=(uint32_t)(SPI1)+0x0c;
	u8 i=3;
	u16 k;

	EECSlo;
	*(__IO uint8_t *) spixbase = 0x90;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up

	while(i--){
	*(__IO uint8_t *) spixbase = 0;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	*(__IO uint8_t *) spixbase;//pick it up
	}

	*(__IO uint8_t *) spixbase = 0;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	k=(u16)(*(__IO uint8_t *) spixbase)<<8;//pick it up

	*(__IO uint8_t *) spixbase = 0;
	while (SPI1->SR&SPI_SR_BSY);//wait until not busy
	k+=(u16)(*(__IO uint8_t *) spixbase);//pick it up
	EECShi;
	return k;
}
u16 s25fnumnotclr(void){//this checks how many 256 byte blocks are clear
	u16 blk,j,nz=0;
	for(blk=0;blk<0x4000;blk++){
		s25frd(blk);//puts the data in gbuff
		j=0;
		while(j<256){
			if(gbuff[j]!=0xff)j=257;//breaks out on the first non clear byte
			else j++;
		}
		if(j==257)nz++;
	}
	return nz;
}
void InitIO(void){
	GPIO_InitTypeDef        GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;//led+inductive pwr
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6|GPIO_Pin_0;//w5500cs,s25fcs,w5500rst
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	EECShi;
	WCShi;//w5500 cs

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//w5500 interrupt low
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void InitSPI1(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_Initstructure;

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);//SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);//MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);//MOSI

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//i am not sure the remaining ones are necessary to set
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    SPI_StructInit(&SPI_Initstructure);//before we were modifying the code in spi.c which is no good
    //SPI_Initstructure.SPI_CPHA = SPI_CPHA_1Edge;//this is default
    SPI_Initstructure.SPI_Mode = SPI_Mode_Master;
    SPI_Initstructure.SPI_NSS = SPI_NSS_Soft;
    SPI_Initstructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//runs at 24 mhz 2,4,8,16
	SPI_Init(SPI1,&SPI_Initstructure);

	SPI_Cmd(SPI1,ENABLE);
	SPI_SSOutputCmd(SPI1,DISABLE);
}

void InitUsart2(void){
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef NEWBOARD
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
#else
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
#endif
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate = 115200;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);

	  USART_Cmd(USART2,ENABLE);

}
//this init is designed for the irda version
void InitUsart1(void){
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	  /* Configure USART1 pins:  Rx and Tx ----------------------------*/
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate =IRSPEED;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART1, &USART_InitStructure);

	  USART_SetPrescaler(USART1, 4);//was 2
	  USART_IrDAConfig(USART1, USART_IrDAMode_Normal);
	  USART_IrDACmd(USART1,ENABLE);
	  USART_Cmd(USART1,ENABLE);
}
//looks at the first 32 of gir32buff and calculates the CRC and compares with girbuff[32]-not used
/*
u8 irchkblk(void){
	u32 calccrc;
	CRC->CR |= CRC_CR_RESET;
	calccrc=CRC_CalcBlockCRC(gir32buff,32);
	//xh8(calccrc);
	return (calccrc==gir32buff[32])?0:1;
}
*/
//this routine sends 134 bytes up the irlink-0x7d,128byesdata,4crc,0x7e
//it does it by placing the bytes in g1buff which is 134bytes long
/*
void xirsendblk(void){
	u8 i;
	u32 crc;

	x1c(0x7d);//the beginning byte
	for(i=0;i<32;i++){//the 128 bytes of data
		x1c((u8)(gir32buff[i]>>24));
		x1c((u8)((gir32buff[i]>>16)&0xff));
		x1c((u8)((gir32buff[i]>>8)&0xff));
		x1c((u8)(gir32buff[i]&0xff));
	}
	CRC->CR |= CRC_CR_RESET;
	crc=CRC_CalcBlockCRC(gir32buff,32);
	x1c((u8)(crc>>24));
	x1c((u8)((crc>>16)&0xff));
	x1c((u8)((crc>>8)&0xff));
	x1c((u8)(crc&0xff));
	x1c(0x7e);
}
*/
void xdp(char c){//xdp just prints teh character is parentheses around it
	xdc('(');
	xdc(c);
	xdc(')');
}
void xques(){
	xdc('?');
	xdf(PROMPT);
}
u8 xdf(const char* pptr){
	u8 len;
	if(gdcnt==GDLEN)return 0;//before we try to put anything on, we have to check if the buffer is full

	len=strlen(pptr);
	if(gdcnt+len>GDLEN) len=GDLEN-gdcnt;
	strncpy(gdbuff+gdcnt,pptr,len);//tack it onto the end
	gdcnt+=len;
	return len;//added
}
void xirc(char x){
	if(giroutcnt<GIROUTLEN){//don't let it overflow-throw away the char
		*(giroutbuff+(giroutcnt++))=x;
	}
}
void xdc(char x){
	if(gdcnt!=GDLEN){
		*(gdbuff+(gdcnt++))=x;
	}
}
void xh(u8 x){//called with 0-f prints "0"-"f"
	xdc((char)(x+(x<10?0x30:55)));
}
void xh2(u8 x){
	xh(x>>4);
	xh(x&0xf);
}
void xh4(u16 x){
	xh2((u8)(x>>8));
	xh2((u8)(x&0xff));
}
void xh8(u32 x){
	xh4((u16)(x>>16));
	xh4((u16)(x&0xffff));
}
void xb3(u16 x,int8_t* buff){
	u8 i;
	if(x>999)x=999;
	i=x/100;
	buff[0]=i?i+0x30:' ';
	x-=i*100;
	buff[1]=(x/10)+0x30;
	buff[2]=(x%10)+0x30;
}
void xb4(u16 x,int8_t* buff,u8 spaceleadingzero){
	u8 i;
	if(x>9999)x=9999;
	i=x/1000;
	buff[0]=!i?' ':i+0x30;
	x-=i*1000;
	i=x/100;
	buff[1]=i+0x30;
	x-=i*100;
	buff[2]=(x/10)+0x30;
	buff[3]=(x%10)+0x30;
}
void xb5(u16 x,u8* buff){
	u8 i;
	i=x/10000;
	buff[0]=i+0x30;
	x-=i*10000;
	i=x/1000;
	buff[1]=i+0x30;
	x-=i*1000;
	i=x/100;
	buff[2]=i+0x30;
	x-=i*100;
	buff[3]=(x/10)+0x30;
	buff[4]=(x%10)+0x30;
}
u8 xb5s(u16 x,int8_t*buff){//returns the number of digits to print
	u8 i=x/10000,p=0;
	if(i) buff[p++]=i+0x30;
	x-=i*10000;
	i=x/1000;
	if(i || p) buff[p++]=i+0x30;
	x-=i*1000;
	i=x/100;
	if(i || p) buff[p++]=i+0x30;
	x-=i*100;//now x is between 0-99
	if(x>9 || p) buff[p++]=(x/10)+0x30;
	buff[p++]=(x%10)+0x30;//always show at least 1 0
	return p;
}
void xb2(u8 x,int8_t* buff,u8 format){//format==0:we put leading zero,format==1:leading space
	u8 i;
	if(x>99)x=99;
	i=x/10;
	buff[0]=i?i+0x30:(!format?'0':' ');
	buff[1]=(x%10)+0x30;
}
void xd2(u8 x){//supresses 1st digit if zero
	u8 i,j;
	i=x/100;
	if(i){
		xdc(i+0x30);
		x-=i*100;
	}
	j=x/10;
	if(i||j)xdc(j+0x30);
	xdc((x%10)+0x30);
}
void xd3(u8 x){//always put out three digits
	u8 i;
	i=x/100;
	xdc(i+0x30);
	x-=i*100;
	xdc((x/10)+0x30);
	xdc((x%10)+0x30);
}
void xds(u8 x){
	if(x&0x80){
		xdc('-');
		x^=0xff;
		x+=1;
	}
	xd2(x);
}
void xdshort(short x){
	if(x<0){
		x=-x;
		xdc('-');
		}
	xd5(x);
}
void xd5(u16 x){//supresses 1st 3 digits if all are zero
	u8 i,j,k;

	i=x/10000;
	if(i){
		xdc(i+0x30);
		x-=i*10000;
	}

	j=x/1000;
	if(j || i){
		xdc(j+0x30);
		x-=j*1000;
	}

	k=x/100;
	if(i||j||k){
		xdc(k+0x30);
		x-=k*100;
	}

	xdc((x/10)+0x30);
	xdc((x%10)+0x30);
}
void wait100(void){
	volatile u16 y=430;
	while(--y);
}
void wait(u16 x){//set for 100us per step
	if(!x) return;
	while(x--) wait100();
}
void wait5micro(u16 x){
	volatile u16 y;
	while(x--){
		y=21;
		while(--y);
	}
}

int main(void)
{
u8 i,ledon=0;
u16 k,m;
u16 wrxwptr,wrxrptr;


  SysTick_Config(6000000);  //48mhz clock 8x per second

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//used by comp1 and i2cfastmodeplus

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_CRC, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_USART2, ENABLE);//this enables the clock to the power block

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_USART1, ENABLE);

  InitIO();
  InitSPI1();
  InitUsart2();
  InitUsart1();

  INDUCTIVEon;

  xdf(PROMPT);
  xdf(HELLO);
#ifdef ETHER
  xdf(ETHERSTR);
#endif

  xdc(REV0);
  xdc(REV1);
  xdc(REV2);
  xdc(REV3);
  xdf(PROMPT);

#ifdef ETHER //if not ETHER then we don't do anything with the w5500
  WRSThi;//release it only if ETHER
#endif

  wait(1000);

#ifdef ETHER
//InitCMR();//writes my ip address + gateway + mac?
//InitS0CONFIG();//sets the destination ip address 192.168.1.35
//InitS0MR();//puts us in tcp mode
//InitS0CR(1);//open a connection
#endif

while (1){
if(gtimer){//goes to 1 8 times per second
	gtimer=0;
	if(++ledon==8){
		LEDon;
		ledon=0;
		gseconds++;//this is a 16 bit counter that overflows
		if(gstate){
			MQTTpublish();
		}
	}
	else LEDoff;
}

	if(giroutcnt && USART1->ISR&USART_FLAG_TXE){//IR transmit port-with the new code, we
		  USART_SendData(USART1, *giroutbuff);//the bottom char goes out
		  if(--giroutcnt)for(i=0;i<giroutcnt;i++) giroutbuff[i]=giroutbuff[i+1];//we need to look at which of these is faster
	}

	if(gdcnt && USART2->ISR&USART_FLAG_TXE){//chk if chars in buffer 1st and then xmit buffer is empty, put one out
	  	USART_SendData(USART2, *gdbuff);//the bottom char goes out
	  	if(--gdcnt)for(i=0;i<gdcnt;i++) gdbuff[i]=gdbuff[i+1];//we need to look at which of these is faster
	}

	  //this is the ir receive buffer-! indicates the start of a 17 byte packet and # indicates the end
	  if(USART1->ISR & USART_FLAG_RXNE){
		  i=(u8)(USART1->RDR & 0xFF);//extract the char from the buffer
		  //xdc('r');
		  if(i=='!'){//we always start a new packet when we receive '!'
			  girincnt=1;
			  girinbuff[0]='!';
		  }
		  else if(i=='#'){
			  if(girincnt==16){
				  girinbuff[16]='#';
				  xirack();
#ifdef ETHER
				  wsendpkt(girinbuff,17);
#else
				  for(i=0;i<17;i++)xdc(girinbuff[i]);//send the packet out the serial port
				  //xdc('\n');
				  //xdc('\r');
#endif
			  }
			  girincnt=0;//purge the buffer
		  }
		  else{
			  if(girincnt) girinbuff[girincnt++]=i;//don't put stuff in the buffer unless ! was detected
		  }
	  }
	  if(USART2->ISR & USART_FLAG_RXNE){//lastly do we have any chars in input buffer
	  	i=(u8)(USART2->RDR & 0xFF);
	  	if(i!=0xd && i!='t')xdc(i);//the t invokes the test mode and there is no acknowledment
	  	switch(i){
	  			case 'a'://writes 19 bytes moderegister(1) ip gateway(4), ip mask(4), mac(6),my ip address(4)
	  		  	InitCMR();
	  		  	InitS0CONFIG();
	  		  	InitS0MR();//1 byte puts us in tcp mode
	  		  	InitS0CR(1);
	  		  	i=0;
	  		  	k=gseconds+10; //this might overflow
	  		  	while(!i){
	  		  		wrc(S0SR,gbuff,1);//socket 0 status register read
	  		  		if(gbuff[0]==0x13)i=1;
	  		  		else if(gseconds==k)i=2;//gives us 30 seconds to establish a connection
	  		  	}
	  		  	if(i==2)
	  		  	{
	  		  		xdf(FAILED);
	  		  		xdf(TCPOPEN);
	  		  	}
	  		  	else
	  		  	{
	  		  	xdf(TCPOPEN);

	  		  	InitS0CR(4); //connect to the IP address and port in intis0config
	  		  	i=0;
	  		  	k=gseconds+30; //this might overflow
	  		  	while(!i){
	  		  		wrc(S0SR,gbuff,1);//socket 0 status register read
	  		  		if(gbuff[0]==0x17)i=1;
	  		  		else if(gseconds==k)i=2;//gives us 30 seconds to establish a connection
	  		  	}
	  		  	if(i==2)
	  		  	{
	  		  		xdf(FAILED);
	  		  		xdf(TCPCONNECTION);
	  		  	}
	  		  	else
	  		  	{
	  		  		xdf(TCPCONNECTION);
	  		  		MQTTconnect(3);
	  		  		wait(5);//500usecs
	  		  		wrc(S0SR,gbuff,1);//socket 0 status register read
	  		  		if(gbuff[0]!=0x17)
	  		  		{
	  		  			xdf(FAILED);//we need to change this routine so it returns 0 if the connection was accepted
	  		  			xdf(MQTTCONNECTION);
	  		  		}
	  		  		else
	  		  		{
	  		  			xdf(MQTTCONNECTION);//next we have to check that an accept was returned
	  		  			i=0;
	  		  			k=gseconds+2;
	  		  			while(!i)
	  		  			{
	  		  				wrc(S0RXRCV,gbuff,2);//socket 0 number of bytes received
	  		  				m=((u16)(gbuff[0])<<8) + (u16)gbuff[1];
	  		  				if(m==4)i=1;
	  		  				else if (k==gseconds)i=2;
	  		  			}
	  		  			if(i==2)//if we get here we didn't get the right number of bytes or any bytes
	  		  			{
	  		  				xdf(MQTTEXPECTED);
	  		  				xd5(m);
	  		  			}
	  		  			else
	  		  			{//now we got 4 bytes pick them up and check them
	  		  				wrc(S0RX,gbuff,4);
	  		  				if(gbuff[0]==0x20 && gbuff[1]==2 && !gbuff[3])
	  		  				{
	  		  					xdf(MQTTACCEPTED);
	  		  					xxxxxxgstate=1;//start sending publish packets
	  		  				}
	  		  				else//print out the response
	  		  				{
	  		  					xdf(MQTTRESPONSE);
	  		  					for(i=0;i<4;i++)
	  		  					{
	  		  						xh2(gbuff[i]);
	  		  						xdc(' ');
	  		  					}
	  		  					xdf(CRLF);
	  		  				}

	  		  			}
	  		  		}
	  		  	}
	  		  	}
	  		  	break;

	  			case 's'://check the socket 0 status register
	  			wrc(S0SR,gbuff,1);//socket 0 status register read
	  			xh2(gbuff[0]);
	  			break;

	  			case 'r'://the number of bytes of received data
				wrc(S0RXRCV,gbuff,2);//socket 0 number of bytes received
				k=((u16)(gbuff[0])<<8) + (u16)gbuff[1];
	  			xh4(k);
				break;

	  		  	case 'p'://a sample publish packet
	  		  	MQTTpublish();
	  		  	break;

	  		  	case 'i'://a ping packet
	  		  	gbuff[0]=0xc;
	  		  	gbuff[1]=0;
	  		  	wsendpkt(gbuff,2);
	  		  	break;


	  		  	case 'A'://reads 'a' back
	  			wrc(CMR,gbuff,19);
	  			for(i=0;i<19;i++){
	  			 xh2(gbuff[i]);
	  			 xdc(' ');
	  			}
	  			break;

	  			/*
	  			case 'B'://reads the phy register-we need to keep polling this until we get a link up
	  			wrc(0x2e,0,gbuff,1);
	  			xh2(gbuff[0]);
	  			xdc('-');
	  			wrc(0x15,0,gbuff,1);
	  			xh2(gbuff[0]);
	  			break;
	  			*/


	  		  	case 'E'://reads the data back from the TX buffer
	  		  	wrc(S0TX,gbuff,20);
	  		  	for(i=0;i<20;i++){
	  		  		xh2(gbuff[i]);
	  		  		xdc(' ');
	  		  	}
	  		  	break;

	  		  	case 'F'://reads txFSR TXWPTR and TXRPTR
	  		  	//wrc(S0TXSIZ.gbuff,1);
	  		  	//xh2(gbuff[0]);
	  		  	//xdc(':');
	  		  	wrc(S0TXFSR,gbuff,2);
	  		  	k=(u16)(gbuff[0])<<8;
	  		  	k+=(u16)gbuff[1];
	  		  	xh4(k);
	  		  	xdc('-');
	  		  	wrc(S0TXWPTR,gbuff,2);
	  		  	k=(u16)(gbuff[0])<<8;
	  		  	k+=(u16)gbuff[1];
	  		  	xh4(k);
	  		  	xdc('-');
	  		  	wrc(S0TXRPTR,gbuff,2);
	  		  	k=(u16)(gbuff[0])<<8;
	  		  	k+=(u16)gbuff[1];
	  		  	xh4(k);
	  		  	break;

	  		  	case 'I':
	  		  	gbuff[0]=0x1f;//should clear the interrupt bits
	  		  	wwc(S0IR,gbuff,1);
	  		  	break;

	  		  	case 'j'://check the rx read pointer and write pointer, the write pointer should be incremented if a packet came in
	  		  	wrc(S0RXRPTR,gbuff,2);
	  		  	wrxrptr=(u16)(gbuff[0])<<8;
	  		  	wrxrptr+=gbuff[1];
	  		  	xh4(wrxrptr);
	  		  	xdc('-');
	  		  	wrc(S0RXWPTR,gbuff,2);
	  		  	wrxwptr=(u16)(gbuff[0])<<8;
	  		  	wrxwptr+=gbuff[1];
	  		  	xh4(wrxwptr);
	  		  	break;

	  		  	case 'J':
	  		  	wrc(S0RX,gbuff,16);
	  		  	for(i=0;i<16;i++){
	  		  	xh2(gbuff[i]);
	  		  	xdc(' ');
	  		  	}
	  		  	break;

	  		  	case 'K'://complete the read
	  		  	gbuff[0]=0x40;
	  		  	wwc(S0CR,gbuff,1);
	  		  	break;

	  		  	case 'v'://read the chip version register
	  		  	wrc(0x39,0,gbuff,1);
	  		  	xh2(gbuff[0]);
	  		    	break;


	  	case 0xd:
	  	break;
	  	default:
	  	xdc('?');
	  	break;
	  	}
	  	xdf(PROMPT);
	  }//end RXNE
  }
}

/*

	    	case 'A'://reads 'a' back
	  			wrc(CMR,gbuff,19);
	  			for(i=0;i<19;i++){
	  			 xh2(gbuff[i]);
	  			 xdc(' ');
	  			}
	  			break;
	  	case 'T':
	  	xirack();
	  	break;

	  	case 'b'://this should write data to be transmitted into the tx buffer
	  	InitS0CONFIGBC();//change to broadcast address
	  	gbuff[0]=0x31;
	  	gbuff[1]=0x32;
	  	gbuff[2]=0x33;
	  	gbuff[3]=0x34;
	  	gbuff[4]='A';
	  	gbuff[5]='B';
	  	gbuff[6]='C';
	  	gbuff[7]='D';
	  	wwc(S0TX,gbuff,8);

	  	gbuff[0]=0;
	  	gbuff[1]=8;
	  	wwc(S0TXWPTR,gbuff,2);

	  	gbuff[0]=0x20;
	  	wwc(S0CR,gbuff,1);
	  	InitS0CONFIG();
	  	break;

	  	case 'i':
	  		  	wrc(S0IR,gbuff,1);
	  		  	xh2(gbuff[0]);
	  		  	gbuff[0]=0xff;
	  		  	wwc(S0IR,gbuff,1);
	  		  	break;

		case 'b':
		xdc('s');
		xdc('1');
		xdc(':');
		xh2(s25fstat(0x5));
		xdc(' ');

		xdc('s');
		xdc('2');
		xdc(':');
		xh2(s25fstat(0x35));
		xdc(' ');

		xdc('s');
		xdc('3');
		xdc(':');
		xh2(s25fstat(0x33));
		break;

		case 'B':
		xh4(s25fid());
		break;

		case 'c':
		xh4(s25fnumnotclr());
		break;

		case 'C':
		s25ferall();
		while(s25fstat(0x5)&1);
		xdc('L');
		xdc('R');
		break;

		case 'd':
		s25frd(0);
		for(j=0;j<256;j++){
		if(gbuff[j]!=0xff){
		xh2(gbuff[i]);
		xdc(' ');
		}
		}
		break;
*/

/******************************************************************************/
/* GLCD_SPI_LPC1700.c: LPC1700 low level Graphic LCD (320x240 pixels) driven  */
/*                     with SPI functions                                     */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2009 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/


#include <lpc17xx.h>
#include "LCD.h"
#include "Font_24x16.h"

  static unsigned short DeviceCode;
/*********************** Hardware specific configuration **********************/

/* 8bit to 16bit LCD Interface 
   
   PINS: 
   - EN      = P0.19
   - LE      = P0.20
   - DIR     = P0.21
   - CS      = P0.22
   - RS      = P0.23 
   - WR      = P0.24 
   - RD      = P0.25
   - DB[0.7] = P2.0...P2.7
   - DB[8.15]= P2.0...P2.7                                                     */

#define PIN_EN		(1 << 19)
#define PIN_LE		(1 << 20)
#define PIN_DIR		(1 << 21)
#define PIN_CS      (1 << 22)
#define PIN_RS		(1 << 23)
#define PIN_WR		(1 << 24)
#define PIN_RD		(1 << 25)   

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    18

/*---------------------- Graphic LCD size definitions ------------------------*/

#define WIDTH       320                 /* Screen Width (in pixels)           */
#define HEIGHT      240                 /* Screen Hight (in pixels)           */
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Pin EN setting to 0 or 1                                                   */
#define LCD_EN(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_EN) : (LPC_GPIO0->FIOCLR = PIN_EN));
/* Pin LE setting to 0 or 1                                                   */
#define LCD_LE(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_LE) : (LPC_GPIO0->FIOCLR = PIN_LE));
/* Pin DIR setting to 0 or 1                                                   */
#define LCD_DIR(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_DIR) : (LPC_GPIO0->FIOCLR = PIN_DIR));
/* Pin CS setting to 0 or 1                                                   */
#define LCD_CS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_CS) : (LPC_GPIO0->FIOCLR = PIN_CS));
/* Pin RS setting to 0 or 1                                                   */
#define LCD_RS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_RS) : (LPC_GPIO0->FIOCLR = PIN_RS));
/* Pin WR setting to 0 or 1                                                   */
#define LCD_WR(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_WR) : (LPC_GPIO0->FIOCLR = PIN_WR));
/* Pin RD setting to 0 or 1                                                   */
#define LCD_RD(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_RD) : (LPC_GPIO0->FIOCLR = PIN_RD));
 
/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short TextColor = Black, BackColor = White;


/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) {

  cnt <<= DELAY_2N;
  while (cnt--);
}

__asm void wait()
{
    nop
    BX lr
}

void wait_delay(int count)
{
  while(count--);
}

/*******************************************************************************
* Send 1 byte over serial communication                                        *
*   Parameter:    byte:   byte to be sent                                      *
*   Return:                                                                    *
*******************************************************************************/

static __inline unsigned char lcd_send (unsigned short byte) {

  LPC_GPIO2->FIODIR |= 0x000000ff;  //P2.0...P2.7 Output
  LCD_DIR(1)		   				//Interface A->B
  wait();
  LCD_EN(0)	                        //Enable 2A->2B
  LPC_GPIO2->FIOPIN =  byte;        //Write D0..D7
  LCD_LE(1)
  wait();                           
  LCD_LE(0)							//latch D0..D7
  LPC_GPIO2->FIOPIN =  byte >> 8;   //Write D8..D15 
  wait();
  return(1);
}


/*******************************************************************************
* read 1 byte over serial communication                                        *
*   Parameter:    byte:   byte to be sent                                      *
*   Return:                                                                    *
*******************************************************************************/

static __inline unsigned short lcd_read (void) {
  unsigned short id;
  LPC_GPIO2->FIODIR &= 0xffffff00;                //P2.0...P2.7 Input
  LCD_DIR(0)		   				              //Interface B->A
  LCD_EN(0)	                                      //Enable 2B->2A
  wait_delay(50);							      //delay some times
  id = LPC_GPIO2->FIOPIN & 0x00ff;                //Read D8..D15                         
  LCD_EN(1)	                                      //Enable 1B->1A
  wait_delay(50);							      //delay some times
  id = (id << 8) | (LPC_GPIO2->FIOPIN & 0x00ff);  //Read D0..D7                         
  LCD_DIR(1)						
  return(id); 
}

/*******************************************************************************
* Write command to LCD controller                                              *
*   Parameter:    c:      command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_cmd (unsigned char c) {

  LCD_RS(0)
  LCD_RD(1)
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}


/*******************************************************************************
* Write data to LCD controller                                                 *
*   Parameter:    c:      data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat (unsigned short c) {

  LCD_RS(1)
  LCD_RD(1)
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}

/*******************************************************************************
* Read data from LCD controller                                                *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/

static __inline unsigned short rd_dat (void) {
  unsigned short val = 0;

  LCD_RS(1)
  LCD_WR(1)
  LCD_RD(0)
  val = lcd_read();
  LCD_RD(1)
  return val;
}

/*******************************************************************************
* Start of data writing to LCD controller                                      *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_start (void) {

  LCD_RS(1)
}


/*******************************************************************************
* Stop of data writing to LCD controller                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_stop (void) {

  LCD_CS(1)
}


/*******************************************************************************
* Data writing to LCD controller                                               *
*   Parameter:    c:      data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_only (unsigned short c) {

  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}


/*******************************************************************************
* Write to LCD register                                                        *
*   Parameter:    reg:    register to be read                                  *
*                 val:    value to write to register                           *
*******************************************************************************/

static __inline void wr_reg (unsigned char reg, unsigned short val) {

  LCD_CS(0)
  wr_cmd(reg);
  wr_dat(val);
  LCD_CS(1)
}


/*******************************************************************************
* Read from LCD register                                                       *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from register                             *
*******************************************************************************/

static unsigned short rd_reg (unsigned short reg) {
  unsigned short val = 0;

  LCD_CS(0)
  wr_cmd(reg);
  val = rd_dat(); 
  LCD_CS(1)
  return (val);
}


/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller                                        *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Init (void) { 
  /* Configure the LCD Control pins */

  LPC_GPIO2->FIODIR   |= 0x000000FF;
  LPC_GPIO0->FIODIR   |= 0x03f80000;					/*P0.19--P0.25           		*/
  LPC_GPIO0->FIOSET    = 0x03f80000;					/*设置P0.19--P0.25 为输出		*/

  delay(5);                            				 	/* Delay 50 ms           		*/

  DeviceCode = rd_reg(0x00);

	if(DeviceCode==0x9325||DeviceCode==0x9328)			/*ILI9325	  大屏9320 小屏9325*/
	{
		wr_reg(0x00e7,0x0010);      
        wr_reg(0x0000,0x0001);
        wr_reg(0x0001,0x0100);     
        wr_reg(0x0002,0x0700);                   
        wr_reg(0x0003,(1<<12)|(3<<4)|(0<<3) );    
        wr_reg(0x0004,0x0000);                                   
        wr_reg(0x0008,0x0207);	           
        wr_reg(0x0009,0x0000);         
        wr_reg(0x000a,0x0000);         
        wr_reg(0x000c,0x0001);         
        wr_reg(0x000d,0x0000);          
        wr_reg(0x000f,0x0000);

        wr_reg(0x0010,0x0000);   
        wr_reg(0x0011,0x0007);
        wr_reg(0x0012,0x0000);                                                                 
        wr_reg(0x0013,0x0000);                 
        delay(5); 
        wr_reg(0x0010,0x1590);   
        wr_reg(0x0011,0x0227);
        delay(5); 
        wr_reg(0x0012,0x009c);                  
        delay(5); 
        wr_reg(0x0013,0x1900);   
        wr_reg(0x0029,0x0023);
        wr_reg(0x002b,0x000e);
        delay(5); 
        wr_reg(0x0020,0x0000);                                                            
        wr_reg(0x0021,0x013f);           
		delay(5); 
        wr_reg(0x0030,0x0007); 
        wr_reg(0x0031,0x0707);   
        wr_reg(0x0032,0x0006);
        wr_reg(0x0035,0x0704);
        wr_reg(0x0036,0x1f04); 
        wr_reg(0x0037,0x0004);
        wr_reg(0x0038,0x0000);        
        wr_reg(0x0039,0x0706);     
        wr_reg(0x003c,0x0701);
        wr_reg(0x003d,0x000f);
        delay(5); 

        wr_reg(0x0050,0x0000);
        wr_reg(0x0051,0x00ef);                   
        wr_reg(0x0052,0x0000);                   
        wr_reg(0x0053,0x013f); 
        
        wr_reg(0x0060,0xa700);        
        wr_reg(0x0061,0x0001); 
        wr_reg(0x006a,0x0000);
        wr_reg(0x0080,0x0000);
        wr_reg(0x0081,0x0000);
        wr_reg(0x0082,0x0000);
        wr_reg(0x0083,0x0000);
        wr_reg(0x0084,0x0000);
        wr_reg(0x0085,0x0000);
      
        wr_reg(0x0090,0x0010);     
        wr_reg(0x0092,0x0000);  
        wr_reg(0x0093,0x0003);
        wr_reg(0x0095,0x0110);
        wr_reg(0x0097,0x0000);        
        wr_reg(0x0098,0x0000);  
		   
        wr_reg(0x0007,0x0133);   
        wr_reg(0x0020,0x0000);                                                            
        wr_reg(0x0021,0x013f); 
	}else if(DeviceCode==0x9320||DeviceCode==0x9300)
	{
    /* Start Initial Sequence --------------------------------------------------*/
	wr_reg(0x00, 0x0001);                 /* Start internal OSC                 */
	wr_reg(0x01, 0x0100);                 /* Set SS and SM bit                  */
	wr_reg(0x02, 0x0700);                 /* Set 1 line inversion               */
	wr_reg(0x03, 0x1038);                 /* Set GRAM write direction and BGR=1 */
	wr_reg(0x04, 0x0000);                 /* Resize register                    */
	wr_reg(0x08, 0x0202);                 /* 2 lines each, back and front porch */
	wr_reg(0x09, 0x0000);                 /* Set non-disp area refresh cyc ISC  */
	wr_reg(0x0A, 0x0000);                 /* FMARK function                     */
	wr_reg(0x0C, 0x0000);                 /* RGB interface setting              */
	wr_reg(0x0D, 0x0000);                 /* Frame marker Position              */
	wr_reg(0x0F, 0x0000);                 /* RGB interface polarity             */
	
	/* Power On sequence -------------------------------------------------------*/
	wr_reg(0x10, 0x0000);                 /* Reset Power Control 1              */
	wr_reg(0x11, 0x0000);                 /* Reset Power Control 2              */
	wr_reg(0x12, 0x0000);                 /* Reset Power Control 3              */
	wr_reg(0x13, 0x0000);                 /* Reset Power Control 4              */
	delay(20);                            /* Discharge cap power voltage (200ms)*/
    wr_reg(0x10, 0x17B0);                 /* SAP, BT[3:0], AP, DSTB, SLP, STB   */
	wr_reg(0x11, 0x0137);                 /* DC1[2:0], DC0[2:0], VC[2:0]        */
	delay(5);                             /* Delay 50 ms                        */
	wr_reg(0x12, 0x0139);                 /* VREG1OUT voltage                   */
	delay(5);                             /* Delay 50 ms                        */
	wr_reg(0x13, 0x1D00);                 /* VDV[4:0] for VCOM amplitude        */
	wr_reg(0x29, 0x0013);                 /* VCM[4:0] for VCOMH                 */
	delay(5);                             /* Delay 50 ms                        */
	wr_reg(0x20, 0x0000);                 /* GRAM horizontal Address            */
	wr_reg(0x21, 0x0000);                 /* GRAM Vertical Address              */
	
	/* Adjust the Gamma Curve --------------------------------------------------*/
	wr_reg(0x30, 0x0006);
	wr_reg(0x31, 0x0101);
	wr_reg(0x32, 0x0003);
	wr_reg(0x35, 0x0106);
	wr_reg(0x36, 0x0B02);
	wr_reg(0x37, 0x0302);
	wr_reg(0x38, 0x0707);
	wr_reg(0x39, 0x0007);
	wr_reg(0x3C, 0x0600);
	wr_reg(0x3D, 0x020B);
	  
	/* Set GRAM area -----------------------------------------------------------*/
	wr_reg(0x50, 0x0000);                 /* Horizontal GRAM Start Address      */
	wr_reg(0x51, (HEIGHT-1));             /* Horizontal GRAM End   Address      */
	wr_reg(0x52, 0x0000);                 /* Vertical   GRAM Start Address      */
	wr_reg(0x53, (WIDTH-1));              /* Vertical   GRAM End   Address      */
	wr_reg(0x60, 0x2700);                 /* Gate Scan Line                     */
	wr_reg(0x61, 0x0001);                 /* NDL,VLE, REV                       */
	wr_reg(0x6A, 0x0000);                 /* Set scrolling line                 */
	
	/* Partial Display Control -------------------------------------------------*/
	wr_reg(0x80, 0x0000);
	wr_reg(0x81, 0x0000);
	wr_reg(0x82, 0x0000);
	wr_reg(0x83, 0x0000);
	wr_reg(0x84, 0x0000);
	wr_reg(0x85, 0x0000);
	
	/* Panel Control -----------------------------------------------------------*/
	wr_reg(0x90, 0x0010);
	wr_reg(0x92, 0x0000);
	wr_reg(0x93, 0x0003);
	wr_reg(0x95, 0x0110);
	wr_reg(0x97, 0x0000);
	wr_reg(0x98, 0x0000);	   
  wr_reg(0x07, 0x0137);                 /* 262K color and display ON          */ 
	}
	else if(DeviceCode==0x8999)	   //对应的驱动IC为SSD1289
	{
		//************* Start Initial Sequence **********//
		wr_reg(0x00, 0x0001); // Start internal OSC.
		wr_reg(0x01, 0x3B3F); // Driver output control, RL=0;REV=1;GD=1;BGR=0;SM=0;TB=1
		wr_reg(0x02, 0x0600); // set 1 line inversion
		//************* Power control setup ************/
		wr_reg(0x0C, 0x0007); // Adjust VCIX2 output voltage
		wr_reg(0x0D, 0x0006); // Set amplitude magnification of VLCD63
		wr_reg(0x0E, 0x3200); // Set alternating amplitude of VCOM
		wr_reg(0x1E, 0x00BB); // Set VcomH voltage
		wr_reg(0x03, 0x6A64); // Step-up factor/cycle setting  7b30
		//************ RAM position control **********/
		wr_reg(0x0F, 0x0000); // Gate scan position start at G0.
		wr_reg(0x44, 0xEF00); // Horizontal RAM address position
		wr_reg(0x45, 0x0000); // Vertical RAM address start position
		wr_reg(0x46, 0x013F); // Vertical RAM address end position
		// ----------- Adjust the Gamma Curve ----------//
		wr_reg(0x30, 0x0000);
		wr_reg(0x31, 0x0706);
		wr_reg(0x32, 0x0206);
		wr_reg(0x33, 0x0300);
		wr_reg(0x34, 0x0002);
		wr_reg(0x35, 0x0000);
		wr_reg(0x36, 0x0707);
		wr_reg(0x37, 0x0200);
		wr_reg(0x3A, 0x0908);
		wr_reg(0x3B, 0x0F0D);
		//************* Special command **************/
		wr_reg(0x28, 0x0006); // Enable test command
		wr_reg(0x2F, 0x12EB); // RAM speed tuning
		wr_reg(0x26, 0x7000); // Internal Bandgap strength
		wr_reg(0x20, 0xB0E3); // Internal Vcom strength
		wr_reg(0x27, 0x0044); // Internal Vcomh/VcomL timing
		wr_reg(0x2E, 0x7E45); // VCOM charge sharing time  
		//************* Turn On display ******************/
		wr_reg(0x10, 0x0000); // Sleep mode off.
		delay(8); // Wait 30mS
		wr_reg(0x11, 0x6878); // Entry mode setup. 262K type B, take care on the data bus with 16it only 
		wr_reg(0x07, 0x0033); // Display ON	*/
	}
}


/*******************************************************************************
* Set draw window region to whole screen                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_WindowMax (void) {
		wr_reg(0x50, 0);                      /* Horizontal GRAM Start Address      */
		wr_reg(0x51, HEIGHT-1);               /* Horizontal GRAM End   Address (-1) */
		wr_reg(0x52, 0);                      /* Vertical   GRAM Start Address      */
		wr_reg(0x53, WIDTH-1);                /* Vertical   GRAM End   Address (-1) */
}

void LCD_WindowMax (unsigned int x,unsigned int y,unsigned int x_end,unsigned int y_end) {

	if(DeviceCode==0x8999)
	{
	wr_reg(0x44,x|((x_end-1)<<8));
	wr_reg(0x45,y);
	wr_reg(0x46,y_end-1);
	}
	else
	{
		wr_reg(0x50, x);                      /* Horizontal GRAM Start Address      */
		wr_reg(0x51, x_end-1);               /* Horizontal GRAM End   Address (-1) */
		wr_reg(0x52, y);                      /* Vertical   GRAM Start Address      */
		wr_reg(0x53, y_end-1);                /* Vertical   GRAM End   Address (-1) */
	}
}
/*******************************************************************************
* Draw a pixel in foreground color                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

void LCD_PutPixel (unsigned int x, unsigned int y) {

  wr_reg(0x20, y);
  wr_reg(0x21, WIDTH-1-x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat(TextColor);
  LCD_CS(1)
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:      color:    foreground color                                 *
*   Return:                                                                    *
*******************************************************************************/

void LCD_SetTextColor (unsigned short color) {

  TextColor = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:      color:    background color                                 *
*   Return:                                                                    *
*******************************************************************************/

void LCD_SetBackColor (unsigned short color) {

  BackColor = color;
}

/*
*********************************************************************************************************
* Description: 	Setting the coordinate of cursor
* Arguments  : 	(Xpos , Ypos) the coordinate
* Returns    : 	None
*********************************************************************************************************
*/
void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos)
{
 	if(DeviceCode==0x8999||DeviceCode==0x9919)
	{
		wr_reg(0x004E, Xpos);
		wr_reg(0X004F, Ypos);
	}
	else 
	{
	wr_reg(0x20, Xpos);
	wr_reg(0x21, Ypos);
	}
}
/*******************************************************************************
* Clear display                                                                *
*   Parameter:      color:    display clearing color                           *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Clear (unsigned short color) {
  unsigned int   i;

  GLCD_WindowMax();

  LCD_SetCursor(0,0);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for(i = 0; i < (WIDTH*HEIGHT); i++)
    wr_dat_only(color);
  wr_dat_stop();
}


/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void LCD_DrawChar (unsigned int x, unsigned int y, unsigned short *c) {
  int idx = 0, i, j;

  x = WIDTH-x-CHAR_W;
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+CHAR_H-1);             /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+CHAR_W-1);             /* Vertical   GRAM End   Address (-1) */

  LCD_SetCursor(y,x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < CHAR_H; j++) {
    for (i = CHAR_W-1; i >= 0; i--) {
      if((c[idx] & (1 << i)) == 0x00) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
    c++;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   c:        ascii character                                  *
*   Return:                                                                    *
*******************************************************************************/

void LCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char c) {

  c -= 32;
  LCD_DrawChar(col * CHAR_W, ln * CHAR_H, (unsigned short *)&Font_24x16[c * CHAR_H]);
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   s:        pointer to string                                *
*   Return:                                                                    *
*******************************************************************************/

void LCD_DisplayString (unsigned int ln, unsigned int col, unsigned char *s) {

  GLCD_WindowMax();
  while (*s) {
    LCD_DisplayChar(ln, col++, *s++);
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:      ln:       line number                                      *
*   Return:                                                                    *
*******************************************************************************/

void LCD_ClearLn (unsigned int ln) {

  GLCD_WindowMax();
  LCD_DisplayString(ln, 0, "                    ");
}

/*******************************************************************************
* Draw bargraph                                                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        maximum width of bargraph (in pixels)            *
*                   val:      value of active bargraph (in 1/1024)             *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Bargraph (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val) {
  int i,j;

  x = WIDTH-x-w;
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+CHAR_H-1);             /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  val = (val * w) >> 10;                /* Scale value for 24x12 characters   */
  LCD_SetCursor(y,x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (i = 0; i < h; i++) {
    for (j = w-1; j >= 0; j--) {
      if(j >= val) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

  x = WIDTH-x-w;
  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, y+h-1);                  /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  LCD_SetCursor(y,x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < h; j++) {
    bitmap_ptr += w-1;
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
    bitmap_ptr += w+1;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bmp file image at position x horizontally and y vertically *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bmp:      address at which the bmp data resides            *
*   Return:                                                                    *
*******************************************************************************/

void LCD_Bmp (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bmp) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bmp;

  x = WIDTH-x-w;
//  wr_reg(0x50, y);                      /* Horizontal GRAM Start Address      */
//  wr_reg(0x51, y+h-1);                  /* Horizontal GRAM End   Address (-1) */
//  wr_reg(0x52, x);                      /* Vertical   GRAM Start Address      */
//  wr_reg(0x53, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  LCD_WindowMax(y,x,y+h,x+w);

  LCD_SetCursor(y,x);
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  bitmap_ptr += (h*w)-1;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
  }
  wr_dat_stop();
}

/******************************************************************************/

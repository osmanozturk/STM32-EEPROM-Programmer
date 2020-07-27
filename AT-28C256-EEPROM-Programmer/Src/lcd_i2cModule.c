/**
*@file: lcd_i2cModule.c
*@brief: Character lcd i2c stm32 hal driver
*@author: Veysel Gökdemir, � 2018
*@note: This library is compatible with the LCD Displays that have similarities(pinouts, conf., ddram address...) with HD44780, KS0066U, etc.
* The library has been tested on the lcd 16x2.
*/

/*Includes---------------------------------------------------------*/
#include "lcd_i2cModule.h"
#include "lcd_userConf.h"
#include <string.h>

/*-----------------------------------------------------------------*/

/*Defines and variables--------------------------------------------*/
#ifdef LCD_16x2
#define line_MAX  2
#define chr_MAX  16

static const uint8_t Cursor_Data[line_MAX][chr_MAX] = {
  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}, //1. line DDRAM address
  {0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F}, //2. line DDRAM address
};
#endif

#ifdef LCD_16x4
static const uint8_t line_MAX = 4;
static const uint8_t chr_MAX = 16;

static const uint8_t Cursor_Data[line_MAX][chr_MAX] = {
  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}, //1. line DDRAM address
  {0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F}, //2. line DDRAM address
	{0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F}, //3. line DDRAM address
  {0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F}, //4. line DDRAM address
};
#endif

#ifdef LCD_20x2
static const uint8_t line_MAX = 2;
static const uint8_t chr_MAX = 20;

static const uint8_t Cursor_Data[line_MAX][chr_MAX] = {
  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13}, //1. line DDRAM address
  {0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53}, //2. line DDRAM address
};
#endif

#ifdef LCD_20x4
static const uint8_t line_MAX = 4;
static const uint8_t chr_MAX = 20;

static const uint8_t Cursor_Data[line_MAX][chr_MAX] = {
  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13}, //1. line DDRAM address
  {0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53}, //2. line DDRAM address
	{0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27}, //3. line DDRAM address
  {0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67}, //4. line DDRAM address
};
#endif

static uint8_t Mask_Data = 0xf0; //Select upper bits.
static uint8_t data[4], data_M, data_L, data_BL;
static uint8_t line_pos = 1; //hold line position, default is 1. line.
static uint8_t str_len = 0; //follow the string lenght.

/*-----------------------------------------------------------------*/

/*Private functions------------------------------------------------*/
/**
*@brief: Lcd i2c device check.
*@retval: none
*/
void LCD_i2cDeviceCheck(void)
{
	/* Checks if target device is ready for communication. */
	/* 3 is number of trials, 1000ms is timeout */
	HAL_Delay(50);
	hi2cx_define();
	while (HAL_I2C_IsDeviceReady(&hi2cx, i2cDeviceAddr, 3, 1000) != HAL_OK) 
	{	
		
	}
}

/**
*@brief: Send commands to lcd.
*@retval: none
*/
void LCD_Set_Command(uint8_t cmd)
{
	data_M = cmd & Mask_Data;        //Most significant bit
	data_L = (cmd << 4) & Mask_Data; //Least significant bit
	
	//For backlight On/off
	data_M |= data_BL;
	data_L |= data_BL;
	
	data[0] = data_M | LCD_E;  //Enable E pin, RS=0
	data[1] = data_M;          //Disable E pin, RS=0
	data[2] = data_L | LCD_E;
  data[3] = data_L;
	
	hi2cx_define();	
	HAL_I2C_Master_Transmit(&hi2cx, i2cDeviceAddr, (uint8_t*)data, 4, 200);
}

/**
*@brief: Write data to lcd.
*@retval: none
*/
void LCD_Write_Data(uint8_t datax)
{
	data_M = datax & Mask_Data;        //Most significant bit
	data_L = (datax << 4) & Mask_Data; //Least significant bit
	
	//For backlight On/off
	data_M |= data_BL;
	data_L |= data_BL;
	 	
	data[0] = data_M | LCD_E|LCD_RS;  //Enable E pin, RS=1
	data[1] = data_M | LCD_RS;        //Disable E pin, RS=1
	data[2] = data_L | LCD_E|LCD_RS;
  data[3] = data_L | LCD_RS;  
	
	hi2cx_define();
	HAL_I2C_Master_Transmit(&hi2cx, i2cDeviceAddr, (uint8_t*)data, 4, 200);
}

/**
*@brief: Clear lcd display.
*@retval: none
*/
void LCD_Clear(void)
{
	LCD_Set_Command(LCD_CLEAR_DISPLAY);
	HAL_Delay(10);
	str_len = 0;
	line_pos = 1;
}

/**
*@brief: Set lcd cursor position.
*@param: line_x: line no, chr_x: character no.
*@retval: none
*/
void LCD_SetCursor(int line_x, int chr_x)
{
  line_pos = line_x; //hold line position.	
	
	if(((line_x >=1 && line_x <= line_MAX) && (chr_x >=1 && chr_x <= chr_MAX)))
	{		
		LCD_Set_Command(LCD_SET_DDRAMADDR | Cursor_Data[line_x - 1][chr_x - 1]);		
	}
}

/**
*@brief: Send string data to lcd.
*@param: str[]: string array, mode: str slide/noslide.
*@retval: none
*/
void LCD_Send_String(char str[], uint8_t mode)
{	 
  static char *buffer[BFR_MAX];
  static uint8_t i[4] = {chr_MAX,chr_MAX,chr_MAX,chr_MAX}; //i follows the ch position while sliding.
  static uint8_t c[4] = {0, 0, 0, 0}; //c follows the each ch of the str buffer while sliding.
  static uint8_t ch_len = 0; //follow the string lenght.
  str_len = 0;
	

	switch(mode)
	{			
		case STR_NOSLIDE:
			
			while (*str) 
			{
				LCD_Write_Data (*str++);
				str_len++;
				if(str_len == chr_MAX)
				{
					LCD_SetCursor(line_pos + 1, 1);
					str_len = 0;
				}			
			}	
			
			break;
		
		case STR_SLIDE:
		
		  for(int a = 0; a < BFR_MAX; a++)
		  buffer[a]=str++;
						
			ch_len = strlen(*buffer);
				 		
			LCD_SetCursor(line_pos, i[line_pos - 1]);	
				  			
			for(int k = c[line_pos - 1];k < ch_len; k++) 
			LCD_Write_Data (*buffer[k]);
								
			i[line_pos - 1]--;
      
			if(i[line_pos -1] == 0)
			{
				i[line_pos - 1] = 1;
				c[line_pos - 1]++;
        if(c[line_pos - 1] == ch_len)
					{					
						i[line_pos - 1] = chr_MAX;
						c[line_pos - 1] = 0;
						ch_len = 0;						
					}												
			}
		 			
			break;	 
	}	
}

/**
*@brief: Print value, ch to lcd.
*@param: *ch: "string + %f", value: float data variable
*@retval: none
*/
void LCD_Print(char const *ch, float value)
{
	char data_ch[BFR_MAX]; //default data size:100.
	
	sprintf(data_ch, ch, value);
	LCD_Send_String(data_ch, STR_NOSLIDE);	
}

/**
*@brief: Backlight control
*@param: light_state: BL on/off
*@retval: none
*/
void LCD_BackLight(uint8_t light_state)
{
	if(light_state == LCD_BL_ON)
	{
    data_BL = LCD_BL_ON;		
		LCD_Write_Data(0x20); //Empty character
	}
	else if (light_state == LCD_BL_OFF)
	{
		data_BL = LCD_BL_OFF;
		LCD_Write_Data(0x20);
	}
}

/**
*@brief: Lcd initiliazing settings.
*@retval: none
*/
void LCD_Init(void)
{
	LCD_Set_Command(LCD_CLEAR_DISPLAY);
	HAL_Delay(1000);
	LCD_Set_Command(LCD_RETURN_HOME);
	HAL_Delay(5);
	LCD_Set_Command(LCD_FUNCTION_SET|MODE_4B|MODE_2L|MODE_5X8_DOTS);
	HAL_Delay(5);
	LCD_Set_Command(LCD_DISPLAY_CONTROL|DISPLAY_ON|CURSOR_OFF|BLINK_OFF);
	HAL_Delay(5);
	LCD_Set_Command(LCD_SET_DDRAMADDR);
	HAL_Delay(500);
}

/*************************END OF FILE*****************************/

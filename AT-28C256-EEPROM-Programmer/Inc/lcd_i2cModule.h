/**
*@file: lcd_i2cModule.h
*@brief: Character lcd i2c stm32 hal driver
*@author: Veysel Gökdemir, � 2018
*/

#ifndef LCD_I2CMODULE_H
#define LCD_I2CMODULE_H 

#ifdef __cplusplus
extern "C" {
#endif

/*Includes-------------------------------------------------------*/
#include "stm32f4xx_hal.h"
	
/*---------------------------------------------------------------*/	
	
/*Defines and variables------------------------------------------*/
//LCD instructions
#define	LCD_CLEAR_DISPLAY           0x01 
#define	LCD_RETURN_HOME             0x02
#define	LCD_ENTRYMODE_SET           0x04
#define	LCD_DISPLAY_CONTROL         0x08
#define	LCD_CURSORDISPLAY_SHIFT     0x10
#define	LCD_FUNCTION_SET            0x20
#define	LCD_SET_CGRAMADDR           0x40
#define	LCD_SET_DDRAMADDR           0x80
//#define	LCD_READ_BUSYFLAG           0x100

//Entry mode set sub-options
typedef enum {
	ENTRY_INCREMENT       = 0x02,
	ENTRY_DECREMENT       = 0x00,
	ENTRY_DISPLAY_SHIFT   = 0x01,
	ENTRY_DISPLAY_NOSHIFT = 0x00
} lcd_EntryMode_set_t;

//Display control sub-options
typedef enum {
	DISPLAY_ON   = 0x04,
	DISPLAY_OFF  = 0x00,
	CURSOR_ON    = 0x02,
	CURSOR_OFF   = 0x00,
	BLINK_ON  = 0x01,
  BLINK_OFF = 0x00	
} lcd_display_control_t;
	
//Cursor or display shift sub-options
typedef enum {
	DISPLAY_SHIFT  = 0x08,
	CURSOR_SHIFT   = 0x00,
	SHIFT_TO_RIGHT = 0x04,
	SHIFT_TO_LEFT  = 0x00
} lcd_CursorDisplay_shift_t;  	

//Function set sub-options
typedef enum {
	MODE_8B        = 0x10,
	MODE_4B        = 0x00,
	MODE_2L        = 0x08,
	MODE_1L        = 0x00,
	MODE_5X10_DOTS = 0x04,
	MODE_5X8_DOTS  = 0x00
} lcd_function_set_t;  

//Read busy flag sub-options
//typedef enum {
//	BUSY_FLAG_ON  = 0x01,
//	BUSY_FLAG_OFF = 0x00
//	} lcd_read_BusyFlag_t; 

//LCD backlight, RS, RW, E definitions for i2c module
//P7............P0 is data that will be sent to lcd through i2c module(PCF8574); 
//P0 : RS
//P1 : RW
//P2 : E
//P3 : BT --> backlight pin activation
//P4 : D4
//P5 : D5
//P6 : D6
//P7 : D7
#define LCD_RS     0x01	
#define LCD_RW     0x02
#define LCD_E      0x04
#define LCD_BL_ON  0x00 //Backlight on
#define LCD_BL_OFF 0x08 //Backlight off
// BACKLIGHT PIN IS REVERSED ON THIS I2C BACKPACK MODULE SO THE VALUES ARE SWITCHED
	
//Define lcd string mode;
#define STR_NOSLIDE 0x00
#define STR_SLIDE 0x01

//Define lcd parameters
//typedef struct {
//	uint8_t i2cAddr;
//	uint8_t X;
//	uint8_t Y;
//	uint8_t Data_Lenght;
//	uint8_t Display_Line;
//	uint8_t Char_Font;
//} lcd_set_device_t;

/*---------------------------------------------------------------*/	

/*Private functions----------------------------------------------*/
/**
*@brief: Lcd i2c device check.
*@retval: none
*/
void LCD_i2cDeviceCheck(void);

/**
*@brief: Lcd initiliazing settings.
*@retval: none
*/
void LCD_Init(void);

/**
*@brief: Clear lcd display.
*@retval: none
*/
void LCD_Clear(void);

///**
//*@brief: LCD display returns to home position.
//*@retval: none
//*/
//void LCD_Return_Home(void);

/**
*@brief: Set lcd cursor position.
*@param: line_x: line no, chr_x: character no.
*@retval: none
*/
void LCD_SetCursor(int line_x, int chr_x);

///**
//*@bief: Read busy flag (BF) indicating internal operation is being performed.
//*@retval: BF
//*/
//HAL_StatusTypeDef LCD_Read_BusyFlag(void);
	
/**
*@brief: Backlight control
*@param: light_state: on/off
*@retval: none
*/
void LCD_BackLight(uint8_t light_state);

/**
*@brief: Send commands to lcd.
*@retval: none
*/
void LCD_Set_Command(uint8_t cmd);

/**
*@brief: Write data.
*@retval: none
*/
void LCD_Write_Data(uint8_t datax);

/**
*@brief: Send string data.
*@param: *str: string pointer, mode: str slide/noslide.
*@retval: none
*/
void LCD_Send_String(char str[], uint8_t mode);

/**
*@brief: Print value, ch to lcd.
*@param: value: uint8_t
*@retval: none
*/
void LCD_Print(char const *ch, float value);

///**
//*@brief: Set lcd parameters.
//*@param: i2cAddr : LCD module(PCF8574) i2c address (default=0x27), char_size: lcd character size, line_size: lcd line size 
//*        data_lenght: lcd 4Bits/8Bits, display_line: lcd line number, char_font: lcd font type. 
//*@retval: none
//*/
//void LCD_SetParam(uint8_t i2cAddr, uint8_t char_size, uint8_t line_size, uint8_t data_length, uint8_t display_line, uint8_t char_font);



	
#ifdef __cplusplus
}
#endif

#endif


/*************************END OF FILE*****************************/

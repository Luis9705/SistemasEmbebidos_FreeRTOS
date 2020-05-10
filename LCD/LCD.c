/*
 * LCD.c
 *
 *  Created on: May 1, 2020
 *      Author: FLL1GA
 */


#include "LCD.h"
#include "miniprintf.h"
#include "i2c.h"

//based and adapted from https://deepbluembedded.com/interfacing-i2c-lcd-16x2-tutorial-with-pic-microcontrollers-mplab-xc8/

unsigned char RS, i2c_add, BackLight_State = LCD_BACKLIGHT;
static const uint8_t SLAVE_ADDRESS_LCD = 0x27; // Use 8-bit address
I2C_HandleTypeDef * hi2c;
/**
 * Initializes the LCD and configures it
 * @param[in] I2C_Add - Address for I2C communication
 */
void LCD_Init(void)
{
  MX_I2C1_Init();
  i2c_add = SLAVE_ADDRESS_LCD << 1;
  hi2c = &hi2c1;

  IO_Expander_Write(0x00);
  HAL_Delay(50);  // wait for >40ms
  LCD_CMD(0x03);
  HAL_Delay(5);
  LCD_CMD(0x03);
  HAL_Delay(1);
  LCD_CMD(0x03);
  HAL_Delay(1);
  LCD_CMD(LCD_RETURN_HOME);
  HAL_Delay(1);
  LCD_CMD(0x20 | (LCD_TYPE << 2));
  HAL_Delay(1);
  LCD_CMD(LCD_TURN_ON);
  HAL_Delay(1);
  LCD_CMD(LCD_CLEAR);
  HAL_Delay(1);
  LCD_CMD(LCD_ENTRY_MODE_SET | LCD_RETURN_HOME);
  HAL_Delay(1);
}

/**
 * Writes 8 bits to the LCD
 * @param[in] Data - 8 bits Data to be written to the LCD
 */
void IO_Expander_Write(unsigned char Data)
{

	unsigned char buff[1] = {Data| BackLight_State};
	HAL_I2C_Master_Transmit (hi2c, i2c_add,(uint8_t *) buff, 1, HAL_MAX_DELAY);

}

/**
 * Writes 8 bits to the LCD in 2 four-bit transactions
 * @param[in] Nibble - 4 bits Data to be written to the LCD
 */
void LCD_Write_4Bit(unsigned char Nibble)
{
  // Get The RS Value To LSB OF Data
  Nibble |= RS;
  IO_Expander_Write(Nibble | 0x04);
  IO_Expander_Write(Nibble & 0xFB);
  HAL_Delay(1);
}

/**
 * Sends a command to the LCD.
 * @param[in] CMD
 */
void LCD_CMD(unsigned char CMD)
{
  RS = 0; // Command Register Select
  LCD_Write_4Bit(CMD & 0xF0);
  LCD_Write_4Bit((CMD << 4) & 0xF0);
}

/**
 * Sends a char to the LCD
 * @param[in] Data
 */
void LCD_Write_Char(char Data)
{
  RS = 1; // Data Register Select
  LCD_Write_4Bit(Data & 0xF0);
  LCD_Write_4Bit((Data << 4) & 0xF0);
}

/**
 * Sends a string to the LCD.
 * @param[in] Str
 */
void LCD_Write_String(char* Str)
{
  for(int i=0; Str[i]!='\0'; i++)
    LCD_Write_Char(Str[i]);
}

/**
 * Sets the cursor in a given position.
 * @param[in] ROW
 * @param[in] COL
 */
void LCD_Set_Cursor(unsigned char ROW, unsigned char COL)
{
  switch(ROW)
  {
    case 2:
      LCD_CMD(0xC0 + COL-1);
      break;
    case 3:
      LCD_CMD(0x94 + COL-1);
      break;
    case 4:
      LCD_CMD(0xD4 + COL-1);
      break;
    // Case 1
    default:
      LCD_CMD(0x80 + COL-1);
  }
}

/**
 * Sends a char to the LCD.
 * @param[in] Str
 */
void LCD_putc(char ch)  {
    LCD_Write_Char(ch);
}

/**
 * Prints UART message
 * param[in] format
 * param[out] rc
 */
int LCD_printf(const char *format, ...)  {
    va_list args;
    int rc;

    va_start(args, format);
    rc = mini_vprintf_cooked(LCD_putc, format, args);
    va_end(args);
    return rc;
}




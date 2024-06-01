/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-sh1106
 */

#ifndef __SH1106_H__
#define __SH1106_H__

#include <stddef.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "sh1106_conf.h"

#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32L5)
#include "stm32l5xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32G0)
#include "stm32g0xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#else
#error "SH1106 library was tested only on STM32F0, STM32F1, STM32F3, STM32F4, STM32F7, STM32L0, STM32L1, STM32L4, STM32H7, STM32G0, STM32G4 MCU families. Please modify sh1106.h if you know what you are doing. Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif

#ifdef SH1106_X_OFFSET
#define SH1106_X_OFFSET_LOWER (SH1106_X_OFFSET & 0x0F)
#define SH1106_X_OFFSET_UPPER ((SH1106_X_OFFSET >> 4) & 0x07)
#else
#define SH1106_X_OFFSET_LOWER 0
#define SH1106_X_OFFSET_UPPER 0
#endif

#include "sh1106_fonts.h"

/* vvv I2C config vvv */

#ifndef SH1106_I2C_PORT
#define SH1106_I2C_PORT        hi2c1
#endif

#ifndef SH1106_I2C_ADDR
#define SH1106_I2C_ADDR        (0x3C << 1)
#endif

/* ^^^ I2C config ^^^ */

/* vvv SPI config vvv */

#ifndef SH1106_SPI_PORT
#define SH1106_SPI_PORT        hspi2
#endif

#ifndef SH1106_CS_Port
#define SH1106_CS_Port         GPIOB
#endif
#ifndef SH1106_CS_Pin
#define SH1106_CS_Pin          GPIO_PIN_12
#endif

#ifndef SH1106_DC_Port
#define SH1106_DC_Port         GPIOB
#endif
#ifndef SH1106_DC_Pin
#define SH1106_DC_Pin          GPIO_PIN_14
#endif

#ifndef SH1106_Reset_Port
#define SH1106_Reset_Port      GPIOA
#endif
#ifndef SH1106_Reset_Pin
#define SH1106_Reset_Pin       GPIO_PIN_8
#endif

/* ^^^ SPI config ^^^ */

#if defined(SH1106_USE_I2C)
extern I2C_HandleTypeDef SH1106_I2C_PORT;
#elif defined(SH1106_USE_SPI)
extern SPI_HandleTypeDef SH1106_SPI_PORT;
#else
#error "You should define SH1106_USE_SPI or SH1106_USE_I2C macro!"
#endif

// SH1106 OLED height in pixels
#ifndef SH1106_HEIGHT
#define SH1106_HEIGHT          64
#endif

// SH1106 width in pixels
#ifndef SH1106_WIDTH
#define SH1106_WIDTH           128
#endif

#ifndef SH1106_BUFFER_SIZE
#define SH1106_BUFFER_SIZE   SH1106_WIDTH * SH1106_HEIGHT / 8
#endif

// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SH1106_COLOR;

typedef enum {
    SH1106_OK = 0x00,
    SH1106_ERR = 0x01  // Generic error.
} SH1106_Error_t;

// Struct to store transformations
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SH1106_t;

typedef struct {
    uint8_t x;
    uint8_t y;
} SH1106_VERTEX;

// Procedure definitions
void sh1106_Init(void);
void sh1106_Fill(SH1106_COLOR color);
void sh1106_UpdateScreen(void);
void sh1106_DrawPixel(uint8_t x, uint8_t y, SH1106_COLOR color);
char sh1106_WriteChar(char ch, FontDef Font, SH1106_COLOR color);
char sh1106_WriteString(char* str, FontDef Font, SH1106_COLOR color);
void sh1106_SetCursor(uint8_t x, uint8_t y);
void sh1106_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SH1106_COLOR color);
void sh1106_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SH1106_COLOR color);
void sh1106_DrawArcWithRadiusLine(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SH1106_COLOR color);
void sh1106_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SH1106_COLOR color);
void sh1106_FillCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SH1106_COLOR par_color);
void sh1106_Polyline(const SH1106_VERTEX *par_vertex, uint16_t par_size, SH1106_COLOR color);
void sh1106_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SH1106_COLOR color);
void sh1106_FillRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SH1106_COLOR color);
void sh1106_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, SH1106_COLOR color);

/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void sh1106_SetContrast(const uint8_t value);

/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void sh1106_SetDisplayOn(const uint8_t on);

/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t sh1106_GetDisplayOn();

// Low-level procedures
void sh1106_Reset(void);
void sh1106_WriteCommand(uint8_t byte);
void sh1106_WriteData(uint8_t* buffer, size_t buff_size);
SH1106_Error_t sh1106_FillBuffer(uint8_t* buf, uint32_t len);

_END_STD_C

#endif // __SH1106_H__

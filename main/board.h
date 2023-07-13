#pragma once

// RGB SPI interface
#define LCD_SPI_DATA0     48  /*!< for 1-line SPI, this also refered as MOSI */
#define LCD_SPI_CLK       45
#define LCD_SPI_CS        38
#define LCD_SPI_DC        -1
#define LCD_PIN_RST       41

#define LCD_VSYNC_GPIO    (41)
#define LCD_HSYNC_GPIO    (42)
#define LCD_DE_GPIO       (40)
#define LCD_PCLK_GPIO     (39)
#define LCD_DATA0_GPIO    (45)  // B0
#define LCD_DATA1_GPIO    (48)  // B1
#define LCD_DATA2_GPIO    (47)  // B2
#define LCD_DATA3_GPIO    (0)  // B3
#define LCD_DATA4_GPIO    (21)  // B4
#define LCD_DATA5_GPIO    (14)  // G0
#define LCD_DATA6_GPIO    (13) // G1
#define LCD_DATA7_GPIO    (12) // G2
#define LCD_DATA8_GPIO    (11) // G3
#define LCD_DATA9_GPIO    (16) // G4
#define LCD_DATA10_GPIO   (17) // G5
#define LCD_DATA11_GPIO   (18) // R0
#define LCD_DATA12_GPIO   (8) // R1
#define LCD_DATA13_GPIO   (3) // R2
#define LCD_DATA14_GPIO   (46) // R3
#define LCD_DATA15_GPIO   (10) // R4
#define LCD_DISP_EN_GPIO  (-1)

#define LCD_PIN_BK_LIGHT       5

#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

#define TP_SDA	15
#define TP_SCL	6


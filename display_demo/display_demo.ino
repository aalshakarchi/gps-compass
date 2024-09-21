/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ThinkInk.h"

#ifdef ARDUINO_ADAFRUIT_FEATHER_RP2040_THINKINK // detects if compiling for
                                                // Feather RP2040 ThinkInk
#define EPD_DC PIN_EPD_DC       // ThinkInk 24-pin connector DC
#define EPD_CS PIN_EPD_CS       // ThinkInk 24-pin connector CS
#define EPD_BUSY PIN_EPD_BUSY   // ThinkInk 24-pin connector Busy
#define SRAM_CS -1              // use onboard RAM
#define EPD_RESET PIN_EPD_RESET // ThinkInk 24-pin connector Reset
#define EPD_SPI &SPI1           // secondary SPI for ThinkInk
#else
#define EPD_DC 6
#define EPD_CS 10
#define EPD_BUSY 8 // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS -1
#define EPD_RESET 7  // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI // primary SPI
#define ENA 9
#endif

// 1.54" Monochrome displays with 200x200 pixels and SSD1681 chipset
ThinkInk_154_Mono_D67 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 1.54" Monochrome displays with 200x200 pixels and SSD1608 chipset
// ThinkInk_154_Mono_D27 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 1.54" Monochrome displays with 152x152 pixels and UC8151D chipset
// ThinkInk_154_Mono_M10 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 2.13" Monochrome displays with 250x122 pixels and SSD1675 chipset
// ThinkInk_213_Mono_B72 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 2.13" Monochrome displays with 250x122 pixels and SSD1675B chipset
// ThinkInk_213_Mono_B73 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 2.13" Monochrome displays with 250x122 pixels and SSD1680 chipset
// ThinkInk_213_Mono_BN display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
// ThinkInk_213_Mono_B74 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
// The GDEY0213B74 is like the B74 above but is not 'shifted down' by 8 pixels
// ThinkInk_213_Mono_GDEY0213B74 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 2.13" Monochrome displays with 212x104 pixels and UC8151D chipset
// ThinkInk_213_Mono_M21 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 2.9" 4-level Grayscale (use mono) displays with 296x128 pixels and IL0373
// chipset
// ThinkInk_290_Grayscale4_T5 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// 2.9" Monochrome displays with 296x128 pixels and UC8151D chipset
// ThinkInk_290_Mono_M06 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 4.2" Monochrome displays with 400x300 pixels and SSD1619 chipset
// ThinkInk_420_Mono_BN display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

// 4.2" Monochrome displays with 400x300 pixels and UC8276 chipset
// ThinkInk_420_Mono_M06 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Adafruit EPD full update test in mono");
  display.begin(THINKINK_MONO);
}

void loop() {
  Serial.println("Banner demo");
  display.clearBuffer();
  Serial.println("Made it this far 1");
  display.setTextSize(2);
  Serial.println("Made it this far 2");
  display.setCursor((display.width() - 180) / 2, (display.height() - 24) / 2);
  Serial.println("Made it this far 3");
  delay(2000);
  display.setTextColor(EPD_BLACK);
  Serial.println("Made it this far 4");
  delay(2000);
  Serial.println("Made it this far 5");
  display.print("Monochrome");
  Serial.println("Made it this far 6");
  display.display();
  Serial.println("Made it this far 7");

  delay(10000);

  Serial.println("B/W rectangle demo");
  Serial.println("Made it this far 8");
  display.clearBuffer();
  display.fillRect(display.width() / 2, 0, display.width() / 2,
                   display.height(), EPD_BLACK);
  display.display();

  delay(10000);

  Serial.println("Text demo");
  // large block of text
  display.clearBuffer();
  display.setTextSize(1);
  testdrawtext(
      "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur "
      "adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, "
      "fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor "
      "neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet "
      "ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a "
      "tortor imperdiet posuere. ",
      EPD_BLACK);
  display.display();

  delay(20000);

  display.clearBuffer();
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(0, 0, i, display.height() - 1, EPD_BLACK);
  }

  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i, EPD_BLACK);
  }
  display.display();

  delay(20000);
}

void testdrawtext(const char *text, uint16_t color) {
  display.setCursor(0, 0);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}
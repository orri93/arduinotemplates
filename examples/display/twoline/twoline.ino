#include <Arduino.h>

#include <gatldisplay.h>

#define TEXT_1 "Test #1"
#define TEXT_2 "Test #2"

::gos::atl::display::Oled<> oled;
::gos::atl::display::line::Two<> twoline(oled);
::gos::atl::buffer::Holder<> buffer1(TEXT_1, sizeof(TEXT_1));
::gos::atl::buffer::Holder<> buffer2(TEXT_2, sizeof(TEXT_2));

void setup() {
  oled.U8g2->begin();
  twoline.display(buffer1, buffer2);
}

void loop() {
  twoline.loop();
}
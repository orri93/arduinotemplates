#include <Arduino.h>

#include <gatldisplay.h>

#define TEXT_1 "Test #1"

::gos::atl::display::Oled<> oled;
::gos::atl::display::line::One<> oneline(oled);
::gos::atl::buffer::Holder<> buffer(TEXT_1, sizeof(TEXT_1));

void setup() {
  oled.U8g2->begin();
  oneline.display(buffer);
}

void loop() {
  oneline.loop();
}
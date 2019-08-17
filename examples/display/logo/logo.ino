#include <gatldisplay.h>
#include "fds-logo.h"

gos::atl::display::Oled<DISPLAY_DEFAULT> oled;

void setup() {
  oled.U8g2.begin();
  oled.logo(fds_logo_width, fds_logo_height, fds_logo_bits);
}

void loop() {
}

#include <gatldisplay.h>
#include "fds-logo.h"

::gos::atl::display::Oled<> oled;

void setup() {
  oled.logo(fds_logo_width, fds_logo_height, fds_logo_bits);
}

void loop() {
}

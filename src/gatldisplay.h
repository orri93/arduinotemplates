#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_DISPLAY_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_DISPLAY_H_

#include <Arduino.h>

#include <Wire.h>
#include <U8g2lib.h>

#include <gatlbuffer.h>

#ifndef DISPLAY_DEFAULT
#define DISPLAY_DEFAULT U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C
#endif

#ifndef DISPLAY_FONT_ONE_LINE
#define DISPLAY_FONT_ONE_LINE u8g2_font_cardimon_pixel_tf
#endif
#ifndef DISPLAY_ONE_LINE_Y
#define DISPLAY_ONE_LINE_Y     31
#endif
#ifndef DISPLAY_ONE_LINE_X
#define DISPLAY_ONE_LINE_X      0
#endif

#ifndef DISPLAY_FONT_TWO_LINES
#define DISPLAY_FONT_TWO_LINES u8g2_font_profont22_mf
#endif
#ifndef DISPLAY_TWO_LINES_Y1
#define DISPLAY_TWO_LINES_Y1   14
#endif
#ifndef DISPLAY_TWO_LINES_Y2
#define DISPLAY_TWO_LINES_Y2   32
#endif


namespace gos {
namespace atl {
namespace display {

template<typename D = DISPLAY_DEFAULT> class Oled {
public:
  Oled() {
    U8g2 = new D(U8G2_R0);
  }
  void logo(u8g2_uint_t w, u8g2_uint_t h, const uint8_t *bitmap) {
    // picture loop
    U8g2->firstPage();
    do {
      U8g2->drawXBMP(
        0,  /* x */
        0,  /* y */
        w,
        h,
        bitmap);
    } while (U8g2->nextPage());
  }
  D* U8g2;
};

template<typename D = DISPLAY_DEFAULT> class Render {
public:
  Render(Oled<D>& oled) : oled_(oled), request_(false), starting_(false) {
  }
  virtual void render(D* d) = 0;
  void loop() {
    if (request_) {
      if (starting_) {
        oled_.U8g2->firstPage();
        starting_ = false;
      }
      render(oled_.U8g2);
      if(!oled_.U8g2->nextPage()) {
        request_ = false;
      }
    }
  }
protected:
  void request() {
    starting_ = request_ = true;
  }
private:
  bool request_;
  bool starting_;
  Oled<D>& oled_;
};

namespace line {

template<typename D = DISPLAY_DEFAULT, typename S = uint8_t>
class One : public Render<D> {
public:
  One(Oled<D>& oled) : Render<D>(oled) {
  }
  void display(::gos::atl::buffer::Holder<S>& holder) {
    holder_ = &holder;
    request();
  }
  void render(D* d) {
    d->setFont(DISPLAY_FONT_ONE_LINE);
    d->drawStr(DISPLAY_ONE_LINE_X, DISPLAY_ONE_LINE_Y, holder_->Buffer);
  }
private:
  Oled<D>& oled_;
  ::gos::atl::buffer::Holder<S>* holder_;
};

template<typename D = DISPLAY_DEFAULT, typename S = uint8_t>
class Two : public Render<D> {
public:
  Two(Oled<D>& oled) : Render<D>(oled) {
  }
  void display(
    ::gos::atl::buffer::Holder<S>& one,
    ::gos::atl::buffer::Holder<S>& two) {
    one_ = &one;
    two_ = &two;
    request();
  }
  void render(D* d) {
    d->setFont(DISPLAY_FONT_TWO_LINES);
    d->drawStr(0, DISPLAY_TWO_LINES_Y1, one_->Buffer);
    d->setFont(DISPLAY_FONT_TWO_LINES);
    d->drawStr(0, DISPLAY_TWO_LINES_Y2, two_->Buffer);
  }
private:
  ::gos::atl::buffer::Holder<S>* one_;
  ::gos::atl::buffer::Holder<S>* two_;
};

}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_DISPLAY_H_ */

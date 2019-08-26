#ifndef _GOS_ARDUINO_TEMPLATE_LIBRARY_DISPLAY_H_
#define _GOS_ARDUINO_TEMPLATE_LIBRARY_DISPLAY_H_

#include <Arduino.h>

#include <Wire.h>
#include <U8g2lib.h>

#include <gatlbuffer.h>
#include <gatlstatistics.h>

#define GOS_FONT_COD_8F                     u8g2_font_pressstart2p_8f
#define GOS_FONT_COD_8R                     u8g2_font_pressstart2p_8r
#define GOS_FONT_COD_8U                     u8g2_font_pressstart2p_8u

#define GOS_FONT_PRO_MF_8                      u8g2_font_profont12_mf
#define GOS_FONT_PRO_MR_8                      u8g2_font_profont12_mr

#define GOS_FONT_PRO_MF_14                     u8g2_font_profont22_mf
#define GOS_FONT_PRO_MR_14                     u8g2_font_profont22_mr

#define GOS_FONT_INC_MF_30                         u8g2_font_inb30_mf
#define GOS_FONT_INC_MR_30                         u8g2_font_inb30_mr

#ifndef DISPLAY_DEFAULT
#define DISPLAY_DEFAULT        U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C
#endif

#ifndef DISPLAY_DEFAULT_FONT
#define DISPLAY_DEFAULT_FONT                          GOS_FONT_COD_8U
#endif

#ifndef DISPLAY_FONT_ONE_LINE
#define DISPLAY_FONT_ONE_LINE                      GOS_FONT_INC_MR_30
#endif
#ifndef DISPLAY_ONE_LINE_Y
#define DISPLAY_ONE_LINE_Y                                         31
#endif
#ifndef DISPLAY_ONE_LINE_X
#define DISPLAY_ONE_LINE_X                                          0
#endif

#ifndef DISPLAY_FONT_TWO_LINES
#define DISPLAY_FONT_TWO_LINES                     GOS_FONT_PRO_MR_14
#endif
#ifndef DISPLAY_TWO_LINES_Y1
#define DISPLAY_TWO_LINES_Y1                                       14
#endif
#ifndef DISPLAY_TWO_LINES_Y2
#define DISPLAY_TWO_LINES_Y2                                       32
#endif

namespace gos {
namespace atl {
namespace display {

typedef u8g2_uint_t u8g2_point_t;

template<class D = DISPLAY_DEFAULT> class Oled {
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

template<class D = DISPLAY_DEFAULT> class Render {
public:
  Render(Oled<D>& oled) :
    oled_(oled),
    request_(false),
    starting_(false) {
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

template<
  class D = DISPLAY_DEFAULT,
  typename I = uint8_t,
  typename P = u8g2_point_t>
class Graph : public Render<D> {
public:
  Graph(Oled<D>& oled, const P& dx = 2) :
    Render<D>(oled),
    points_(nullptr),
    Dx(dx) {
  }
  void display(::gos::atl::statistics::Set<P>& points) {
    points_ = &points;
    request();
  }
  void render(D* d) {
    if (points_->Count > 1) {
      P x = P(0);
      for (I i = I(1); i < points_->Count; i++) {
        d->drawLine(
          x,
          points_->Values[i - 1],
          x + Dx,
          points_->Values[i]);
        x += Dx;
      }
    }
  }
  P Dx;
private:
  ::gos::atl::statistics::Set<P>* points_;
};

template<class D = DISPLAY_DEFAULT>
class String : public Render<D> {
public:
  String(Oled<D>& oled, const uint8_t *font = DISPLAY_FONT_ONE_LINE) :
    Render<D>(oled),
    font_(font) {
  }
protected:
  const uint8_t *font_;
};

namespace line {

template<class D = DISPLAY_DEFAULT, typename S = uint8_t>
class One : public String<D> {
public:
  One(Oled<D>& oled, const uint8_t *font = DISPLAY_FONT_ONE_LINE) : String<D>(oled, font) {
  }
  void display(::gos::atl::buffer::Holder<S>& holder) {
    holder_ = &holder;
    request();
  }
  void render(D* d) {
    d->setFont(String<D>::font_);
    d->drawStr(DISPLAY_ONE_LINE_X, DISPLAY_ONE_LINE_Y, holder_->Buffer);
  }
private:
  ::gos::atl::buffer::Holder<S>* holder_;
};

template<class D = DISPLAY_DEFAULT, typename S = uint8_t>
class Two : public String<D> {
public:
  Two(Oled<D>& oled, const uint8_t *font = DISPLAY_FONT_TWO_LINES) : String<D>(oled, font) {
  }
  void display(
    ::gos::atl::buffer::Holder<S>& one,
    ::gos::atl::buffer::Holder<S>& two) {
    one_ = &one;
    two_ = &two;
    request();
  }
  void render(D* d) {
    d->setFont(String<D>::font_);
    d->drawStr(0, DISPLAY_TWO_LINES_Y1, one_->Buffer);
    d->setFont(String<D>::font_);
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

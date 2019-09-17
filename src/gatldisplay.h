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

namespace details {
template<class D = DISPLAY_DEFAULT>
void logo(D& d, u8g2_uint_t w, u8g2_uint_t h, const uint8_t *bitmap) {
  d.drawXBMP(
    0,  /* x */
    0,  /* y */
    w,
    h,
    bitmap);
}
namespace line {
template<class D = DISPLAY_DEFAULT> void one(
  D& d,
  const char* message,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  d.setFont(font);
  d.drawStr(DISPLAY_ONE_LINE_X, DISPLAY_ONE_LINE_Y, message);
}
template<class D = DISPLAY_DEFAULT, typename S = uint8_t> void one(
  D& d,
  ::gos::atl::buffer::Holder<S>& holder,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  one<D>(d, holder.Buffer, font);
}
template<class D = DISPLAY_DEFAULT> void two(
  D& d,
  const char* messageone,
  const char* messagetwo,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  d->setFont(font);
  d->drawStr(0, DISPLAY_TWO_LINES_Y1, messageone);
  d->setFont(font);
  d->drawStr(0, DISPLAY_TWO_LINES_Y2, messagetwo);
}
template<class D = DISPLAY_DEFAULT, typename S = uint8_t> void two(
  D& d,
  ::gos::atl::buffer::Holder<S>& holderone,
  ::gos::atl::buffer::Holder<S>& holdertwo,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  two<D>(d, holderone.Buffer, holdertwo.Buffer, font);
}
}
}

template<class D = DISPLAY_DEFAULT> class Oled {
public:
  Oled() {
    U8g2 = new D(U8G2_R0);
  }
  D* U8g2;
};

namespace synchronous {
template<class D = DISPLAY_DEFAULT>
void logo(Oled<D>& oled, u8g2_uint_t w, u8g2_uint_t h, const uint8_t *bitmap) {
  // picture loop
  oled.U8g2->firstPage();
  do {
    details::logo<D>(oled.U8g2, w, h, bitmap);
  } while (oled.U8g2->nextPage());
}
namespace line {
template<class D = DISPLAY_DEFAULT>
void one(
  Oled<D>& oled,
  const char* message,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  oled.U8g2->firstPage();
  do {
    details::line::one<D>(oled.U8g2, message, font);
  } while (oled.U8g2->nextPage());
}
template<class D = DISPLAY_DEFAULT, typename S = uint8_t>
void one(
  Oled<D>& oled,
  ::gos::atl::buffer::Holder<S>& holder,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  one<D>(oled, holder, font);
}
template<class D = DISPLAY_DEFAULT>
void two(
  Oled<D>& oled,
  const char* one,
  const char* two,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  oled.U8g2->firstPage();
  do {
    details::line::two<D>(oled.U8g2, one, two, font);
  } while (oled.U8g2->nextPage());
}
template<class D = DISPLAY_DEFAULT, typename S = uint8_t>
void two(
  Oled<D>& oled,
  ::gos::atl::buffer::Holder<S>& one,
  ::gos::atl::buffer::Holder<S>& two,
  const uint8_t *font = DISPLAY_FONT_ONE_LINE) {
  oled.U8g2->firstPage();
  do {
    details::line::two<D>(oled.U8g2, one, two, font);
  } while (oled.U8g2->nextPage());
}
}
}

namespace asynchronous {
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
      if (!oled_.U8g2->nextPage()) {
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

template<class D = DISPLAY_DEFAULT> class Logo : public Render<D> {
public:
  Logo(Oled<D>& oled) : Render<D>(oled), w_(0), h_(0), bitmap_(nullptr) {
  }
  void display(u8g2_uint_t w, u8g2_uint_t h, const uint8_t *bitmap) {
    w_ = w;
    h_ = h;
    bitmap_ = bitmap;
    request();
  }
  void render(D* d) {
    details::logo<D>(*d, w_, h_, bitmap_);
  }
private:
  u8g2_uint_t w_;
  u8g2_uint_t h_;
  const uint8_t *bitmap_;
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
  One(Oled<D>& oled, const uint8_t *font = DISPLAY_FONT_ONE_LINE) :
    String<D>(oled, font) {
  }
  void display(::gos::atl::buffer::Holder<S>& holder) {
    holder_ = &holder;
    request();
  }
  void render(D* d) {
    details::line::one<D,S>(*d, holder_, String<D>::font_);
  }
private:
  ::gos::atl::buffer::Holder<S>* holder_;
};

template<class D = DISPLAY_DEFAULT, typename S = uint8_t>
class Two : public String<D> {
public:
  Two(Oled<D>& oled, const uint8_t *font = DISPLAY_FONT_TWO_LINES) :
    String<D>(oled, font) {
  }
  void display(
    ::gos::atl::buffer::Holder<S>& one,
    ::gos::atl::buffer::Holder<S>& two) {
    one_ = &one;
    two_ = &two;
    request();
  }
  void render(D* d) {
    details::line::two<D, S>(*d, one_, two_, String<D>::font_);
  }
private:
  ::gos::atl::buffer::Holder<S>* one_;
  ::gos::atl::buffer::Holder<S>* two_;
};

}

}

}
}
}

#endif /* _GOS_ARDUINO_TEMPLATE_LIBRARY_DISPLAY_H_ */

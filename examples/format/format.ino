#include <Arduino.h>

#include <gatlformat.h>

#define BUFFER_SIZE 12

#define BAUD 115200

#define TEXT_ID "A:"
#define TEXT_UNIT " C"
#define TEXT_ERROR "Failure"

::gos::atl::format::option::Number option;
::gos::atl::buffer::Holder<> buffer(BUFFER_SIZE);

::gos::atl::buffer::Holder<> id(TEXT_ID, sizeof(TEXT_ID));
::gos::atl::buffer::Holder<> unit(TEXT_UNIT, sizeof(TEXT_UNIT));

static const ::gos::atl::buffer::Holder<>* nullholder = nullptr;

double real = 93.418;
int integer = 666;

void setup() {
  Serial.begin(BAUD);

  ::gos::atl::format::real(buffer, real, option, &id, &unit);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::real(buffer, real, option, &id);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::real(buffer, real, option);
  Serial.println(buffer.Buffer);
  option.Width = 6;
  option.Precision = 2;
  ::gos::atl::format::real(buffer, real, option);
  Serial.println(buffer.Buffer);
  option.Width = -7;
  ::gos::atl::format::real(buffer, real, option);
  Serial.println(buffer.Buffer);
  option.Width = GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE;
  ::gos::atl::format::real(buffer, real, option, &id, &unit);
  Serial.println(buffer.Buffer);

  ::gos::atl::format::integer(
    buffer,
    integer,
    &id,
    &unit,
    GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_UNDEFINED);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::integer(buffer, integer, &id, &unit, 6);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::integer(buffer, integer, &id, nullholder, 6);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::integer(buffer, integer, nullholder, nullholder, 6);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::integer(buffer, integer, &id, &unit);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::integer(buffer, integer);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::integer(
    buffer,
    integer,
    &id,
    &unit,
    GOS_ARDUINO_TEMPLATE_LIBRARY_WIDTH_FILL_NEGATIVE);
  Serial.println(buffer.Buffer);

  ::gos::atl::format::error(buffer, TEXT_ERROR, sizeof(TEXT_ERROR), &id);
  Serial.println(buffer.Buffer);
  ::gos::atl::format::error(buffer, TEXT_ERROR, sizeof(TEXT_ERROR));
  Serial.println(buffer.Buffer);
}

void loop() {
}

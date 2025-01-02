#pragma once
#include <Arduino.h>

struct Point {
  double x;
  double y;

  Point(double x = 0, double y = 0) : x(x), y(y) {}
};

struct MachineState {
  String status;
  struct {
    bool xEndstop;
    bool yEndstop;
  } sensors;
};

struct Error {
  int code;
  String message;
};

struct Response {
  bool success;
  String message;
};
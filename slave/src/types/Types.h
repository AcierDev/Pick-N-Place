#pragma once
#include <Arduino.h>

struct Point {
  double x;
  double y;

  Point(double x = 0, double y = 0) : x(x), y(y) {}
};

struct MachineState {
  String status;
  Point position;
  struct Sensors {
    bool xEndstop = false;
    bool yEndstop = false;
    bool suctionEnabled = false;
    bool armExtended = false;
  } sensors;
  String error;
  bool isHomed = false;

  // Track what changed
  bool statusChanged = false;
  bool positionChanged = false;
  bool sensorsChanged = false;
  bool errorChanged = false;
  bool homedStateChanged = false;

  void clearChangeFlags() {
    statusChanged = false;
    positionChanged = false;
    sensorsChanged = false;
    errorChanged = false;
    homedStateChanged = false;
  }
};

struct Error {
  int code;
  String message;
};

struct Response {
  bool success;
  String message;
};
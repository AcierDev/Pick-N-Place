#include "Protocol.h"

#include <ArduinoJson.h>

void Protocol::sendState(const MachineState& state) {
  StaticJsonDocument<200> doc;
  doc["status"] = state.status;

  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["x_endstop"] = state.sensors.xEndstop;
  sensors["y_endstop"] = state.sensors.yEndstop;

  sendJson("STATE", doc);
}

void Protocol::sendResponse(const Response& response) {
  StaticJsonDocument<200> doc;
  doc["success"] = response.success;
  doc["message"] = response.message;

  String output;
  serializeJson(doc, output);
  Serial.print("RESPONSE ");
  Serial.println(output);
}

void Protocol::debug(const String& message) {
  StaticJsonDocument<100> doc;
  doc["message"] = message;

  sendJson("DEBUG", doc);
}

void Protocol::info(const String& message) {
  StaticJsonDocument<100> doc;
  doc["message"] = message;

  sendJson("INFO", doc);
}

void Protocol::error(const String& message, int code) {
  StaticJsonDocument<100> doc;
  doc["message"] = message;
  doc["code"] = code;

  sendJson("ERROR", doc);
}

void Protocol::sendJson(const String& type, const JsonDocument& doc) {
  Serial.print(type);
  Serial.print(" ");
  serializeJson(doc, Serial);
  Serial.println();
}
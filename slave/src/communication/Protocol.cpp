#include "Protocol.h"

void Protocol::sendState(const MachineState& state) {
  if (state.sensorsChanged) {
    Serial.write(MSG_STATE);
    Serial.print(" sensors=");
    Serial.print(state.sensors.xEndstop ? "1" : "0");
    Serial.print(",");
    Serial.print(state.sensors.yEndstop ? "1" : "0");
    Serial.print(",");
    Serial.print(state.sensors.armExtended ? "1" : "0");
    Serial.print(",");
    Serial.print(state.sensors.suctionEnabled ? "1" : "0");
    Serial.write('\n');
  }

  if (state.positionChanged) {
    sendPosition(state.position.x, state.position.y);
  }

  if (state.statusChanged) {
    Serial.write(MSG_STATE);
    Serial.print(" status=");
    Serial.print(state.status);
    Serial.write('\n');
  }
}

void Protocol::sendResponse(const Response& response) {
  Serial.write(MSG_RESPONSE);
  Serial.print(response.success ? "1 " : "0 ");
  Serial.print(response.message);
  Serial.write('\n');
}

void Protocol::debug(const String& message) {
  Serial.write(MSG_DEBUG);
  Serial.print(message);
  Serial.write('\n');
  Serial.flush();  // Ensure message is sent immediately
}

void Protocol::info(const String& message) {
  Serial.write(MSG_INFO);
  Serial.print(message);
  Serial.write('\n');
  Serial.flush();  // Ensure message is sent immediately
}

void Protocol::error(const String& message, int code) {
  Serial.write(MSG_ERROR);
  Serial.print(code);
  Serial.print(" ");
  Serial.print(message);
  Serial.write('\n');
  Serial.flush();  // Ensure message is sent immediately
}

void Protocol::sendPosition(float x, float y) {
  // Round to 2 decimal places before sending
  x = round(x * 100) / 100.0f;
  y = round(y * 100) / 100.0f;

  uint8_t buffer[9];
  buffer[0] = MSG_POS;
  memcpy(&buffer[1], &x, sizeof(float));
  memcpy(&buffer[5], &y, sizeof(float));
  Serial.write(buffer, sizeof(buffer));
}
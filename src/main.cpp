// #include <Arduino.h>
// #include <Servo.h>

// Servo myservo;
// const int SERVO_PIN = 12;
// int currentAngle = 90;  // Default position

// void setup() {
//   Serial.begin(9600);
//   myservo.attach(SERVO_PIN);

//   // Move to default position
//   myservo.write(currentAngle);

//   Serial.println("Servo Control Ready");
//   Serial.println("Enter angle (0-180) to move servo:");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     String input = Serial.readStringUntil('\n');

//     // Convert input to integer
//     int newAngle = input.toInt();

//     // Validate input
//     if (input.length() > 0 && newAngle >= 0 && newAngle <= 180) {
//       currentAngle = newAngle;
//       myservo.write(currentAngle);
//       Serial.print("Servo moved to ");
//       Serial.print(currentAngle);
//       Serial.println(" degrees");
//     } else {
//       Serial.println("Invalid input! Please enter a number between 0 and
//       180");
//     }

//     Serial.println("Enter angle (0-180) to move servo:");
//   }

//   // Servo will hold its position until new command is received
//   delay(10);  // Small delay to prevent CPU overload
// }
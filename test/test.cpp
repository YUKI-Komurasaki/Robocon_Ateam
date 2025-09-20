#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>
void controlHandForward() {
  Serial.print("Left: "); Serial.print(PS4.Left());
  Serial.print(" | Right: "); Serial.println(PS4.Right());

}

#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
int result;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.write("Hello World");
  Serial.write(result);
  delay(1000);
  Serial.write("MyFunction Result");
  result = myFunction(2, 3);
  Serial.write(result);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
#include <SoftwareSerial.h>
// according to Arduino Docs, the first param is Rx, the second is Tx:
SoftwareSerial bluetooth(6,7); 
#define JOY_1 A0
#define JOY_2 A1
#define ACTUATOR_PIN 3
#define DRILL_PIN 4
#define L_KNOB A2
#define R_KNOB A3
void setup()
{
  bluetooth.begin(9600);   
  Serial.begin(9600);   
  delay(100);
  bluetooth.print("AT+CO094A9A83A52ED");
  delay(1000);
  pinMode(JOY_1, INPUT);
  pinMode(JOY_2, INPUT);
  pinMode(ACTUATOR_PIN, INPUT_PULLUP);
  pinMode(DRILL_PIN, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  int test1 = analogRead(JOY_1);
  int test2 = analogRead(JOY_2);
  int test3 = digitalRead(ACTUATOR_PIN);
  int test4 = digitalRead(DRILL_PIN);
  int trim1 = analogRead(L_KNOB);
  int trim2 = analogRead(R_KNOB);
  const int UPPER = 550;
  const int LOWER = 480;
  
  if ((test1 > LOWER) && (test1 < UPPER)){
    test1 = trim1;
  }

  if ((test2 > LOWER) && (test2 < UPPER)){
    test2 = trim2;
  }
  

  Serial.print(test1);
  Serial.print(", ");
  Serial.println(trim1);

  bluetooth.print(test1);
  bluetooth.print(',');
  bluetooth.print(test2);
  bluetooth.print(',');
  bluetooth.print(test3);
  bluetooth.print(',');
  bluetooth.print(test4);
  /*
  Serial.print(test1);
  Serial.print(',');
  Serial.print(test2);
  Serial.print(',');
  Serial.print(test3);
  Serial.print(',');
  Serial.println(test4);
  */
  delay(100);
}
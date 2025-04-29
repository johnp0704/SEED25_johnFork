// Written by Jack McTasney for UVM SEED Team 24 2024-->2025
// Maybe this will work

#include <SoftwareSerial.h>
#define M1 3  // PWM Capable
#define M2 5  // PWM Capable
#define ACT_UP 4
#define ACT_DOWN 8
#define M_UP 2
#define M_DOWN 9
#define JETSON 10

int values[4];  // Holds values transmitted from joystick and buttons

// according to Arduino Docs, the first param is Rx, the second is Tx:
SoftwareSerial bluetooth(6, 7);

void setup() {
  bluetooth.begin(9600);
  Serial.begin(9600);
  delay(100);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(ACT_UP, OUTPUT);
  pinMode(ACT_DOWN, OUTPUT);
  pinMode(M_UP, OUTPUT);
  pinMode(M_DOWN, OUTPUT);
  pinMode(JETSON, INPUT_PULLUP);
}
void loop() {
  //char data[10] = { '*', '*', '*', '*', '*', '*', '*', '*', '*', '*' };
  // Create array full of *'s
  char data[20];
  char n = '*';
  data[0] = n;
  memcpy(((char *)data) + sizeof(data[0]), data, sizeof(data) - sizeof(data[0]));
  char incoming;
  int i = 0;
  // Get serial buffer into data array
  while ((bluetooth.available() > 0) || (bluetooth.peek() != -1)) {
    incoming = bluetooth.read();
    data[i] = incoming;
    i++;
    delay(1);
  }
  if (digitalRead(JETSON) == LOW) {
    analogWrite(M_DOWN, 50);
    digitalWrite(M_UP, LOW);
    digitalWrite(ACT_UP, LOW);
    digitalWrite(ACT_DOWN, HIGH);
    delay(1000);
    digitalWrite(ACT_UP, LOW);
    digitalWrite(ACT_DOWN, LOW);
    while (digitalRead(JETSON) == LOW) {}
  }
  // Test printing data array
  if (i > 0) {
    for (i = 0; i < 20; i++) {
      if (data[i] == '*') {  // End of data
        i = 100;             // Just cause lol (Will exit loop)
        //Serial.println(" LOL");

        // Extracting Numbers
        int index = 0;
        char *token = strtok(data, ",");      // Get the first token
        while (token != NULL && index < 4) {  // Ensure we don't exceed array bounds
          values[index++] = atoi(token);      // Convert to integer
          token = strtok(NULL, ",");          // Get the next token
        }
        // Print extracted numbers
        /*
        for (int i = 0; i < index; i++) {
          Serial.println(values[i]);
        }
        */
        int M1_val = map(values[0], 0, 1023, 128, 255);
        int M2_val = map(values[1], 0, 1023, 1, 127);



        //analogWrite(M1, values[0]);
        //analogWrite(M2, values[1]);
        Serial.write(M1_val);
        Serial.write(M2_val);
        digitalWrite(ACT_UP, values[2]);
        digitalWrite(ACT_DOWN, !values[2]);
        analogWrite(M_DOWN, !values[3] * 50);
        digitalWrite(M_UP, LOW);
        //digitalWrite(ACT_DOWN, values[3]);

      }  //else
         //Serial.print(data[i]);
    }
  }
}

// #define S0 43
// #define S1 42
// #define S2 48
// #define S3 49
// #define sensorOut 47

// #define S0 PL6
// #define S1 PL7
// #define S2 PL1
// #define S3 PL0
// #define sensorOut PL2
// using port L

// #define S0 41
// #define S1 42
// #define S2 36
// #define S3 35
// #define sensorOut 37

#include "Arduino.h"

// using Port L
#define S0 (1 << 6)  // PL6
#define S1 (1 << 7)  // PL7
#define S2 (1 << 1)  // PL1
#define S3 (1 << 0)  // PL0
#define sensorOut 37 // PL2

void setup()
{

    DDRL = (((S0) | (S1)) | ((S2) | (S3)));
    // setting S0, S1, S2 and S3 pins as input/output

    // setting freq scaling to 20%
    PORTL = ((S0) & ~(S1));
    // setting S0 as HIGH and S1 as LOW

    Serial.begin(9600);
}

void loop()
{

    int frequency = 0;

    // setting RED filtered photodiodes to be read
    PORTL &= (~(S2) & (~S3));
    // setting S2 and S3 to LOW

    // Reading the output frequency
    frequency = pulseIn(sensorOut, LOW);
    // can freq still be programmed this way or need change to bare metal
    // Remaping the value of the frequency to the RGB Model of 0 to 255
    //  frequency = map(frequency, 25, 72, 255, 0);
    //  Printing the value on the serial monitor
    Serial.print("R = ");    // printing name
    Serial.print(frequency); // printing RED color frequency
    Serial.print("  ");
    delay(1000);

    // setting GREEN filtered photodiodes to be read
    PORTL |= ((S2) | (S3));
    // setting S2 and S3 to HIGH

    // Reading the output frequency
    frequency = pulseIn(sensorOut, LOW);
    // can freq still be programmed this way or need change to bare metal
    // Remaping the value of the frequency to the RGB Model of 0 to 255
    // frequency = map(frequency, 25, 72, 255, 0);
    //  Printing the value on the serial monitor
    Serial.print("G = ");    // printing name
    Serial.print(frequency); // printing GREEN color frequency
    Serial.print("  ");
    delay(1000);

    // setting BLUE filtered photodiodes to be read
    PORTL &= (~(S2));
    PORTL |= (S3);
    // setting S2 to LOW and S3 to HIGH

    // Reading the output frequency
    frequency = pulseIn(sensorOut, LOW);
    // can freq still be programmed this way or need change to bare metal
    // Remaping the value of the frequency to the RGB Model of 0 to 255
    // frequency = map(frequency, 25, 72, 255, 0);
    //  Printing the value on the serial monitor
    Serial.print("B = ");    // printing name
    Serial.print(frequency); // printing BLUE color frequency
    Serial.println("  ");
    delay(1000);
}

//---

// void setup() {
//   pinMode(S0, OUTPUT);
//   pinMode(S1, OUTPUT);
//   pinMode(S2, OUTPUT);
//   pinMode(S3, OUTPUT);
//   pinMode(sensorOut, INPUT);

//   // Setting frequency-scaling to 20%
//   digitalWrite(S0, HIGH);
//   digitalWrite(S1, LOW);

//   Serial.begin(9600);
// }

// void loop() {
//   // Setting red filtered photodiodes to be read
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, LOW);
//   // Reading the output frequency
//   frequency = pulseIn(sensorOut, LOW);
//   //Remaping the value of the frequency to the RGB Model of 0 to 255
//   frequency = map(frequency, 25, 72, 255, 0);
//   // Printing the value on the serial monitor
//   Serial.print("R= ");      //printing name
//   Serial.print(frequency);  //printing RED color frequency
//   Serial.print("  ");
//   delay(100);

//   // Setting Green filtered photodiodes to be read
//   digitalWrite(S2, HIGH);
//   digitalWrite(S3, HIGH);
//   // Reading the output frequency
//   frequency = pulseIn(sensorOut, LOW);
//   //Remaping the value of the frequency to the RGB Model of 0 to 255
//   frequency = map(frequency, 30, 90, 255, 0);
//   // Printing the value on the serial monitor
//   Serial.print("G= ");      //printing name
//   Serial.print(frequency);  //printing RED color frequency
//   Serial.print("  ");
//   delay(100);

//   // Setting Blue filtered photodiodes to be read
//   digitalWrite(S2, LOW);
//   digitalWrite(S3, HIGH);
//   // Reading the output frequency
//   frequency = pulseIn(sensorOut, LOW);
//   //Remaping the value of the frequency to the RGB Model of 0 to 255
//   frequency = map(frequency, 25, 70, 255, 0);
//   // Printing the value on the serial monitor
//   Serial.print("B= ");      //printing name
//   Serial.print(frequency);  //printing RED color frequency
//   Serial.println("  ");
//   delay(100);
// }
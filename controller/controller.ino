#include <EEPROM.h>

float kp = 0;
float ki = 0;
float kd = 0;

volatile float t = 0;        // control signal
volatile float accum = 0;    // accumulated error
volatile float diff = 0;     // error difference
volatile float prev = 0;     // previous error
volatile float err = 0;      // error signal
volatile float setpoint = 0; // setpoint
volatile float fb = 0;       // feedback (current motor speed)
volatile int count = 0;      // flag buat interrupt

byte buf[32];

// interrupt service routine setiap 0.1s
ISR(TIMER1_COMPA_vect)
{
    count = 1;
}

void setup()
{
    Serial.begin(115200);

    // penyetingan koefisien-koefisien PID
    Serial.println("CONF");
    String mode = Serial.readStringUntil('\n');
    if (mode == "PERM")
    {
        kp = Serial.parseFloat();
        ki = Serial.parseFloat();
        kd = Serial.parseFloat();

        EEPROM.put(0x00, kp);
        EEPROM.put(0x04, ki);
        EEPROM.put(0x08, kd);
    }
    else if (mode == "TEMP")
    {
        kp = Serial.parseFloat();
        ki = Serial.parseFloat();
        kd = Serial.parseFloat();
    }
    else
    {
        EEPROM.get(0x00, kp);
        EEPROM.get(0x04, ki);
        EEPROM.get(0x08, kd);
    }

    // penyetingan timer 1 untuk interupsi setiap ~10Hz
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 24999; // = 16000000 / (64 * 10) - 1
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void loop()
{
    // pembacaan nilai setpoint
    setpoint = 10 * (float)analogRead(A0) / 1023;

    // prosedur yang dilakukan setiap 0.1s
    if (count == 1)
    {
        while (Serial.available() == 0)
            ;
        fb = Serial.parseFloat();

        prev = err;
        err = setpoint - fb;
        diff = err - prev;
        accum = accum + err;
        t = kp * err + ki * accum + kd * diff;

        Serial.println(t);
        Serial.println(setpoint);

        count = 0;
    }
}
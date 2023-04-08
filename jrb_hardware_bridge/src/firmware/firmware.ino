#include <Adafruit_NeoPixel.h>

const int PIN_STARTER = 23;
const int PIN_TEAM = 22;
const int PIN_STRATEGY = 21;
const int PIN_LED = 11;

const int NB_LED = 10;

Adafruit_NeoPixel LEDmodule = Adafruit_NeoPixel(NB_LED, PIN_LED, NEO_GRB + NEO_KHZ800);  // création de l'objet module

const int BAUDRATE = 115200;
const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
int buffer_index = 0;

bool prevValues[3] = {false, false, false};
bool currentValues[3] = {false, false, false};

unsigned long lastLoopTime = 0;
const unsigned int LOOP_FREQUENCY = 100;                            // Hz
const unsigned long LOOP_PERIOD_MICROS = 1 / LOOP_FREQUENCY * 1000; // us

void readValues()
{
    currentValues[0] = digitalRead(PIN_STARTER);
    currentValues[1] = digitalRead(PIN_TEAM);
    currentValues[2] = digitalRead(PIN_STRATEGY);
}

void sendStarterValue()
{
    Serial.print("d ");
    Serial.print(PIN_STARTER);
    Serial.print(" ");
    Serial.println(currentValues[0]);
}

void sendTeamValue()
{
    Serial.print("d ");
    Serial.print(PIN_TEAM);
    Serial.print(" ");
    Serial.println(currentValues[1]);
}

void sendStrategyValue()
{
    Serial.print("d ");
    Serial.print(PIN_STRATEGY);
    Serial.print(" ");
    Serial.println(currentValues[2]);
}

void sendValues()
{
    sendStarterValue();
    sendTeamValue();
    sendStrategyValue();
}

void emptyBuffer()
{
    // clear the buffer by filling it with null characters
    memset(buffer, 0, sizeof(buffer));
    buffer_index = 0; // reset the buffer index
}

void processSerialData(char *data)
{
    char command;
    char values[BUFFER_SIZE-2];

    sscanf(data, "%c", &command);

    if (strlen(data) > 2) {
        strncpy(values, &data[2], sizeof(values));
        values[sizeof(values) - 1] = '\0'; // make sure buffer is null-terminated
    } else {
        values[0] = '\0'; // empty buffer if string is too short
    }

    switch (command) {
        // Read current values
        case 'r': {
            readValues();
            sendStarterValue();
            sendTeamValue();
            sendStrategyValue();
            break;
        }

        default: {
            Serial.print("l ");
            Serial.print("Unknown command: ");
            Serial.print(command);
            Serial.print(" with values: ");
            Serial.println(values);
        }
    }
}

void readSerial()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read(); // read the next character from the serial port

        if (c == '\n')
        {                                // check if the delimiter has been received
            buffer[buffer_index] = '\0'; // terminate the buffer with a null character
            processSerialData(buffer);   // call the processing function with the received data
            emptyBuffer();               // empty the buffer to make room for new data
        }
        else
        {
            if (buffer_index < BUFFER_SIZE - 1)
            {                             // check if the buffer is not full
                buffer[buffer_index] = c; // add the character to the buffer
                buffer_index++;           // increment the buffer index
            }
            else
            {
                emptyBuffer(); // empty the buffer if it becomes full
                Serial.println("l WARNING buffer full, emptpying it");
            }
        }
    }
}

void setLed()
{
    int R,G,B;
    for(int i=0;i<NB_LED;i++)
    {
        R = random(0, 100);
        G = random(0, 100);
        B = random(0, 100);
        LEDmodule.setPixelColor(i,R,G,B);
    }
    LEDmodule.show();
}

void setup()
{
    Serial.begin(BAUDRATE);

    // Set pin to digital input with internal pullup
    pinMode(PIN_STARTER, INPUT_PULLUP);
    pinMode(PIN_TEAM, INPUT_PULLUP);
    pinMode(PIN_STRATEGY, INPUT_PULLUP);

    // Initial read and send
    readValues();
    sendValues();

    prevValues[0] = currentValues[0];
    prevValues[1] = currentValues[1];
    prevValues[2] = currentValues[2];

    LEDmodule.begin();
/*    for(int i=0;i<NB_LED;i++)
    { 
        LEDmodule.setPixelColor(i,0,0,0);
    }
    LEDmodule.show();*/
    setLed();
}

void loop()
{
    readSerial();

    unsigned long currentMicros = micros();
    if (currentMicros - lastLoopTime < LOOP_PERIOD_MICROS)
    {
        return;
    }

    // Ensure constant frequency of the loop
    lastLoopTime = currentMicros;

    readValues();

    // Check if any of the values have changed and send it if it's the case
    if (prevValues[0] != currentValues[0]) {
        sendStarterValue();
        prevValues[0] = currentValues[0];
    }

    if (prevValues[1] != currentValues[1]) {
        sendTeamValue();
        prevValues[1] = currentValues[1];
    }

    if (prevValues[2] != currentValues[2]) {
        sendStrategyValue();
        prevValues[2] = currentValues[2];
    }
}
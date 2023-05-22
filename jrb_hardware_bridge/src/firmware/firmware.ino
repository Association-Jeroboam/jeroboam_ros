#include <FastLED.h>

FASTLED_USING_NAMESPACE

const int PIN_STARTER = 23;
const int PIN_TEAM = 22;
const int PIN_STRATEGY = 21;
const int DATA_PIN = 11; //LED
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    62
CRGB leds[NUM_LEDS];
#define BRIGHTNESS         255
unsigned int set_led=0;


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
    //unsigned int led,R,G,B;

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
/*
        case 'c':{
            sscanf(values, "%d %d %d %d", &led,&R,&G,&B);
            LEDmodule.setPixelColor(led,R,G,B);
            break;
        }

        case 's':{
            LEDmodule.show();
            Serial.println("a");
            break;
        }
*/
        case 'l':{
            sscanf(values, "%d", &set_led);
            Serial.print("Set led to ");
            Serial.println(set_led);
            break;
        }

        default: {
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

    FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();

SimplePatternList gPatterns = { bpm, juggle };
//SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
  
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

    if (set_led==1)
    {
        gPatterns[gCurrentPatternNumber]();

        for( int i = 0; i < 26; i++){
          leds[i] = CRGB::Lime;
        }

        // send the 'leds' array out to the actual LED strip
        FastLED.show();  
        // insert a delay to keep the framerate modest
        //FastLED.delay(1000/FRAMES_PER_SECOND); 

        // do some periodic updates
        EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
        EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
    }
    else
    {
        for( int i = 0; i < NUM_LEDS; i++){
          //leds[i] = CRGB::Black;
          leds[i] = CRGB::Black;
        }
    }
    FastLED.show();  
}



// LED functions

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
  addGlitter(80);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 55;
  CRGBPalette16 palette = HeatColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
  addGlitter(50);
}

void juggle() {
  // nine colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for( int i = 0; i < 9; i++) {
    leds[beatsin16( i+8, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
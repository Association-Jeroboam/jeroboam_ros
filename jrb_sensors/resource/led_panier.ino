#include <Adafruit_NeoPixel.h>                

#define sortie 3                                           // DATA LED
#define nb_led 30


#define max_value 255
#define min_value 0

Adafruit_NeoPixel module = Adafruit_NeoPixel(nb_led, sortie, NEO_GRB + NEO_KHZ800);  // création de l'objet module

void setup() 
{
  Serial.begin(9600);
  module.begin();

  pinMode(LED_BUILTIN, OUTPUT);
      digit(-1,1);
      digit(-1,0);
      module.show();
}

void digit(int number, int digit_pos)
{
  int offset=digit_pos*15;
  int r=0;
  int g=200;
  int b=10;

  Serial.println(number);
  switch (number)
  {
    case 0 : 
      if(digit_pos==1) for(int i=0;i<=14;i++)
        module.setPixelColor(i+offset,0,0,0); 
      else
      {
        for(int i=0;i<=14;i++) module.setPixelColor(i+offset,r,g,b);     
        module.setPixelColor(6+offset,0,0,0);
        module.setPixelColor(7+offset,0,0,0); 
        module.setPixelColor(8+offset,0,0,0); 
      } 
      break;

    case 1 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,0,0,0);
      for(int i=4;i<=9;i++) module.setPixelColor(i+offset,r,g,b);
      module.setPixelColor(11+offset,r,g,b); 
      module.setPixelColor(14+offset,r,g,b); 

      break;


    case 2 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,r,g,b);
      module.setPixelColor(0+offset,0,0,0);
      module.setPixelColor(3+offset,0,0,0); 
      module.setPixelColor(6+offset,0,0,0); 
      module.setPixelColor(8+offset,0,0,0);
      module.setPixelColor(11+offset,0,0,0);
      module.setPixelColor(12+offset,0,0,0); 

      break;

    case 3 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,0,0,0);
      for(int i=0;i<=5;i++) module.setPixelColor(i+offset,r,g,b);
      module.setPixelColor(14+offset,r,g,b);
      module.setPixelColor(7+offset,r,g,b);
      module.setPixelColor(9+offset,r,g,b);
      module.setPixelColor(10+offset,r,g,b);
      break;

    case 4 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,0,0,0);
      for(int i=0;i<=4;i++) module.setPixelColor(i+offset,r,g,b);
      for(int i=10;i<=12;i++) module.setPixelColor(i+offset,r,g,b);
      module.setPixelColor(7+offset,r,g,b);
      break;

    case 5 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,r,g,b);
      module.setPixelColor(8+offset,0,0,0);
      module.setPixelColor(1+offset,0,0,0); 
      module.setPixelColor(13+offset,0,0,0); 
      module.setPixelColor(6+offset,0,0,0); 
      module.setPixelColor(4+offset,0,0,0);
      break;

    case 6 : 
      for(int i=2;i<=14;i++) module.setPixelColor(i+offset,r,g,b);
      module.setPixelColor(0+offset,0,0,0);
      module.setPixelColor(1+offset,0,0,0); 
      module.setPixelColor(8+offset,0,0,0); 
      module.setPixelColor(6+offset,0,0,0); 
      break;

    case 7 : 
      for(int i=5;i<=14;i++) module.setPixelColor(i+offset,0,0,0);
      for(int i=0;i<=4;i++) module.setPixelColor(i+offset,r,g,b);
      //module.setPixelColor(7+offset,r,g,b);
      module.setPixelColor(9+offset,r,g,b);
      module.setPixelColor(10+offset,r,g,b);
      break;

    case 8 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,r,g,b); 
      module.setPixelColor(8+offset,0,0,0); 
      module.setPixelColor(6+offset,0,0,0); 
      break;

    case 9 : 
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,r,g,b); 
      module.setPixelColor(8+offset,0,0,0); 
      module.setPixelColor(6+offset,0,0,0);
      module.setPixelColor(13+offset,0,0,0); 
      break;

    default :
      for(int i=0;i<=14;i++) module.setPixelColor(i+offset,0,0,0);
      module.setPixelColor(2+offset,50,0,0);
      module.setPixelColor(7+offset,50,0,0);
      module.setPixelColor(12+offset,50,0,0);
      break;
  }
}

void loop() 
{
    if(Serial.available() > 0)
    {
      int score = Serial.parseInt(SKIP_ALL); // can be -1 if read error
      //Serial.flush();

      //int score=12;
     //Serial.println("Score=");
      //Serial.println(score);
      if(score>=2) digitalWrite(LED_BUILTIN, 1);
      else digitalWrite(LED_BUILTIN, 0);
      if(score==-2)
      {
        for(int i=0;i<=29;i++) module.setPixelColor(i,255,0,0);
        module.show();
      }
      else
        if(score!=-1)
        {
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); //toggle led
          digit(int(score/10),1);
          digit(int(score%10),0);
          
          module.show(); 
          module.show(); //des fois ça passe pas du premier coup
        }
    }
}
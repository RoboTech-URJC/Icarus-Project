#include <Thread.h>

//CONSTANTS
enum{
  MaxString = 12,
};

class Lights
{

public:

  Lights()
  {
    Lights::red_led = 3;
    Lights::green_led = 5;
    Lights::blue_led = 6;

    Lights::set_leds();
  }

  void
  run(char state){
    if(state == 'r'){
      
      Lights::turn_on(Lights::red_led);
      Lights::turn_off(Lights::green_led);
      Lights::turn_off(Lights::blue_led);
      
    }else if(state == 'g'){
      
      Lights::turn_on(Lights::green_led);
      Lights::turn_off(Lights::red_led);
      Lights::turn_off(Lights::blue_led);
      
    }else if(state == 'b'){
      
      Lights::turn_on(Lights::blue_led);
      Lights::turn_off(Lights::red_led);
      Lights::turn_off(Lights::green_led);
    }
  }

private:

  int red_led;
  int green_led;
  int blue_led;

  void
  set_leds()
  {
    pinMode(Lights::red_led, OUTPUT);
    pinMode(Lights::green_led, OUTPUT);
    pinMode(Lights::blue_led, OUTPUT);
  }

  void 
  turn_off(int pin_out)
  {
    digitalWrite(pin_out,LOW);
  }

  void 
  turn_on(int pin_out)
  {
    digitalWrite(pin_out, HIGH);    
  }

  void 
  blink_led(int pin_out,float hertz, float t_blinking)
  { 
    float interval = (1.0/hertz)*1000.0;
    long i = millis()+ t_blinking * 1000.0;
    
    while(millis()<i){
      
      turn_on(pin_out);
      delay(interval);
      
      turn_off(pin_out);
      delay(interval);
    }
  
    turn_off(pin_out);
      
  }
  
};

Lights lights;

void 
setup() 
{
  Serial.begin(9600);
}

char state;
void 
loop() 
{
  /*
   * Implements reactive algorithm
   */
  
  //read state
  state = Serial.read();
  Serial.println(state);
  //exec state
  lights.run(state);
  delay(50);
  
}

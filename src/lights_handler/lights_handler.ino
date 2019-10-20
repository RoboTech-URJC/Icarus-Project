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
    if(state == 'a'){
      
      Lights::turn_off(Lights::red_led);
      Lights::turn_on(Lights::green_led);
      Lights::turn_off(Lights::blue_led);
      
    }else if(state == 'b'){
      
      Lights::turn_on(Lights::red_led);
      Lights::turn_off(Lights::green_led);
      Lights::turn_off(Lights::blue_led);
      
    }else if(state == 'c'){
      
      Lights::turn_on(Lights::blue_led);
      Lights::turn_off(Lights::red_led);
      Lights::turn_off(Lights::green_led);
    }else if(state == 'p'){
      Lights::turn_off(Lights::red_led);
      Lights::turn_off(Lights::green_led);
      Lights::turn_off(Lights::blue_led);
      Lights::blink_led(Lights::blue_led, 5);
    }
  }

  void
  sleep(unsigned long t0, float freq)
  {
    int dead_time;
    unsigned long tf, elapsed;
    tf = millis();
    elapsed = tf - t0;
    dead_time = (int) ((1.0 / freq) - elapsed);
    if(dead_time > 0){
      delay(dead_time);
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
    digitalWrite(pin_out, LOW);
  }

  void 
  turn_on(int pin_out)
  {
    digitalWrite(pin_out, HIGH);    
  }

  void 
  blink_led(int pin_out, float hertz)
  { 
    float interval = (1.0 / hertz) * 1000.0;
    
    turn_on(pin_out);
    delay(interval);
    
    turn_off(pin_out);
    delay(interval);
      
  }
  
};

Lights lights;

void 
setup() 
{
  Serial.begin(9600);
}

char state = ' ';
unsigned long t0;
void 
loop() 
{
  /*
   * Implements reactive algorithm
   */
  
  //read state
  t0 = millis();
  state = Serial.read();
  //Serial.println(state);
  //exec state
  lights.run(state);
  lights.sleep(t0, 12.0);
  
}

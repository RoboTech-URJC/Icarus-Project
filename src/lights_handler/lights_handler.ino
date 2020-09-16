class Lights
{

public:

  Lights()
  {
    Lights::red_led_head = 3;
    Lights::green_led_head = 5;
    Lights::blue_led_head = 6;

  Lights::red_led_tail = 9;
    Lights::green_led_tail = 10;
    Lights::blue_led_tail = 11;

    Lights::set_leds();
  }

  void
  run(char state){
    if(state == 'a'){   //activate/arm mode = green lights

    lights_status = 'a';
      Lights::turn_off(Lights::red_led_head);
      Lights::turn_on(Lights::green_led_head);
      Lights::turn_off(Lights::blue_led_head);

    Lights::turn_off(Lights::red_led_tail);
    Lights::turn_on(Lights::green_led_tail);
    Lights::turn_off(Lights::blue_led_tail);

   }else if(state == 'b'){  //flying mode = blue lights

    lights_status = 'b';
      Lights::turn_on(Lights::red_led_head);
      Lights::turn_off(Lights::green_led_head);
      Lights::turn_off(Lights::blue_led_head);

    Lights::turn_on(Lights::red_led_tail);
    Lights::turn_off(Lights::green_led_tail);
    Lights::turn_off(Lights::blue_led_tail);

  }else if(state == 'c'){   //warning =  red lights

    lights_status = 'c';
      Lights::turn_on(Lights::blue_led_head);
      Lights::turn_off(Lights::red_led_head);
      Lights::turn_off(Lights::green_led_head);

    Lights::turn_on(Lights::blue_led_tail);
    Lights::turn_off(Lights::red_led_tail);
    Lights::turn_off(Lights::green_led_tail);

  }else if(state == 'p'){   // pulse = flash lights

    lights_status = 'p';
      Lights::turn_off(Lights::red_led_head);
      Lights::turn_off(Lights::green_led_head);
      Lights::turn_off(Lights::blue_led_head);

    Lights::turn_off(Lights::red_led_tail);
      Lights::turn_off(Lights::green_led_tail);
      Lights::turn_off(Lights::blue_led_tail);


      Lights::blink_led(Lights::blue_led_head, pulse_interval);
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

  int red_led_head;
  int green_led_head;
  int blue_led_head;

  int red_led_tail;
  int green_led_tail;
  int blue_led_tail;


  int pulse_interval = 2; //interval of the flashing lights
  char lights_status; //current status of lights

  void
  set_leds()
  {
    pinMode(Lights::red_led_head, OUTPUT);
    pinMode(Lights::green_led_head, OUTPUT);
    pinMode(Lights::blue_led_head, OUTPUT);

  pinMode(Lights::red_led_tail, OUTPUT);
    pinMode(Lights::green_led_tail, OUTPUT);
    pinMode(Lights::blue_led_tail, OUTPUT);
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
  while (lights_status = 'p') {

      turn_on(pin_out);
      delay(interval);

      turn_off(pin_out);
      delay(interval);
  }
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

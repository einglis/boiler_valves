#include <Arduino.h>
#include <Adafruit_SleepyDog.h>

// ----------------------------------------------------------------------------

namespace outputs {
  enum {
    status_led = LED_BUILTIN, // (aka 13 on Arduino Nano)

    valve_hw_pin  =  8,
    valve_ch1_pin =  9,
    valve_ch2_pin = 10,
    valve_ch3_pin = 11,
    boiler_pin    = 12,

    valve_hw_led  = A3,
    valve_ch1_led = A2,
    valve_ch2_led = A1,
    valve_ch3_led = A0,
    boiler_led    = A4,
  };
}
namespace inputs {
  enum {
    control_hw_pin  = 4,
    control_ch1_pin = 3,
    control_ch2_pin = 5,
    control_ch3_pin = 6,
  };
}

const int  open_time_ms = 8000; // seems tremendously slow, but the valves are actally even slower to open fully
const int close_time_ms = 4000;
const long overrun_time_ms = 5 * 60 * 1000L;
  // five minutes, the length of time we hold the overrun valve open once all demand is gone
const long underrun_time_ms = close_time_ms + 1000;
  // the time we wait before allowing the demand signal to propagate to the boiler

// ----------------------------------------------------------------------------

unsigned int pattern_phase = 0x0001;
  // rolled every 100ms for status

unsigned int min_brightness = 0x0101; // less frequent than 1 in 8 gives a noticable flicker
unsigned int mid_brightness = 0x1111;
  // rolled every millisecond for PWM

unsigned int roll_right( unsigned int x ) { return (x >> 1) | (x << 15); }

void pattern( int pin, unsigned int pattern )
{
  digitalWrite( pin, !!(pattern & pattern_phase) ); // !! as patterns are 16-bit but digitalWrite takes an 8-bit.
}

// ----------------------------------------------------------------------------

class Channel
{
public:
  Channel( int input_pin, int output_pin, int led_pin, const char* tag )
    : tag{ tag }
    , in_pin{ input_pin }
    , out_pin{ output_pin }
    , led_pin{ led_pin }
    , demand_count{ 0 }
    , demand{ false }
    , open_count{ 0 }
    , close_count{ 0 }
  { }

  void setup()
  {
    pinMode( in_pin, INPUT );
    pinMode( out_pin, OUTPUT );
    pinMode( led_pin, OUTPUT );
  }

  void poll()
  {
    const int in = digitalRead( in_pin );
    if (in && demand_count < demand_count_max)
      demand_count++;     // unequal debounce with 512 steps up...
    else if (!in && demand_count > 0)
      demand_count >>= 1; // ...9 steps down

    if (demand_count == demand_count_max)
      demand = true;
    else if (demand_count == 0)
      demand = false;

    if (open_count > 0 && open_count < open_count_max) // opening
    {
      pattern( led_pin, 0xff00 & mid_brightness );
      open_count++;
    }
    else if (close_count > 0 && close_count < close_count_max) // closing
    {
      pattern( led_pin, 0x3333 & mid_brightness );
      close_count++;
    }
    else // open or closed
    {
      digitalWrite( led_pin, open_count > 0 );
    }
  }

  void open()
  {
    digitalWrite( out_pin, true );
    if (open_count == 0)
      open_count = 1;  // get the ball rolling
    close_count = 0;
  }

  void close()
  {
    digitalWrite( out_pin, false );
    if (close_count == 0)
      close_count = 1;
    open_count = 0;
  }

  bool has_demand() { return demand; }
  bool fully_open() { return open_count == open_count_max; }

  const char *const tag;

private:
  const int in_pin;
  const int out_pin;
  const int led_pin;

  long demand_count;
  enum { demand_count_max = 512 }; // magic number; half a second demand -> on; 9 ms no demand -> off
  bool demand;

  long open_count;  // counts up
  enum { open_count_max = open_time_ms };
  long close_count;  // counts up
  enum { close_count_max = close_time_ms };
};

Channel hw  = { inputs::control_hw_pin,  outputs::valve_hw_pin,  outputs::valve_hw_led,  "HW"  };
Channel ch1 = { inputs::control_ch1_pin, outputs::valve_ch1_pin, outputs::valve_ch1_led, "CH1" };
Channel ch2 = { inputs::control_ch2_pin, outputs::valve_ch2_pin, outputs::valve_ch2_led, "CH2" };
Channel ch3 = { inputs::control_ch3_pin, outputs::valve_ch3_pin, outputs::valve_ch3_led, "CH3" };
Channel *channels[] = { &hw, &ch1, &ch2, &ch3 };
Channel *overrun_ch = &ch1; // the valve that gets opened on the overrun

// ideally, ch3 would be the default choice, since that'll be the towel rads
// but it's likely this'll be in use before that's plumbed, so needs to work
// safely with only one heating valve: ch1.

// ----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Boiler valve manager");

  pinMode( outputs::status_led, OUTPUT );
  pinMode( outputs::boiler_pin, OUTPUT );
  pinMode( outputs::boiler_led, OUTPUT );

  for (auto chan : channels)
    chan->setup();

  Watchdog.enable(5000);
    // five second watchdog, petted every few seconds by the status LED loop
}

// ----------------------------------------------------------------------------

enum State { idle, demand, overrun };
State curr_state = State::idle;

void my_state( State state )
{
  if (state != curr_state)
  {
    switch (state)
    {
      case idle:    Serial.println(F("NEW STATE: idle"));    break;
      case demand:  Serial.println(F("NEW STATE: demand"));  break;
      case overrun: Serial.println(F("NEW STATE: overrun")); break;
    }
    curr_state = state;
  }
}

static size_t bits_to_tags( char *buf, unsigned int x )
{
  buf[0] = (x & 1) ? 'W' : '-';
  buf[1] = (x & 2) ? '1' : '-';
  buf[2] = (x & 4) ? '2' : '-';
  buf[3] = (x & 8) ? '3' : '-';
  return 4;
}

static void report_state_change(
  unsigned int this_demand, unsigned int last_demand,
  unsigned int this_open, unsigned int last_open )
{
  char buf[128];
  char *bp = &buf[0];
  bp += sprintf(bp, "Change of state; demand ");
  bp += bits_to_tags(bp, last_demand);
  bp += sprintf(bp, " -> ");
  bp += bits_to_tags(bp, this_demand);
  bp += sprintf(bp, ", open ");
  bp += bits_to_tags(bp, last_open);
  bp += sprintf(bp, " -> ");
  bp += bits_to_tags(bp, this_open);
  *bp = '\0';
  Serial.println(buf);
}

// ----------------------------------------------------------------------------

long overrun_counter_ms = 0;   // counts down
long underrun_counter_ms = 0;  // counts down

// ------------------------------------

void loop()
{
  static unsigned long last = millis();
  unsigned long now = millis();
  if (now == last)
    return;

  min_brightness = roll_right(min_brightness); // update PWMs
  mid_brightness = roll_right(mid_brightness);
  last = now;


  static unsigned long pattern_last = now;
  if (now - pattern_last > 100)
  {
    pattern_phase = roll_right(pattern_phase);
    pattern_last = now;

    if (pattern_phase == 1)
      Watchdog.reset();
  }

  pattern( outputs::status_led, 0x0001 & min_brightness );


  unsigned int this_demand = 0;
  unsigned int this_open = 0;

  int index = 0;
  for (auto chan : channels)
  {
    chan->poll();
    if (chan->has_demand()) this_demand |= (1 << index);
    if (chan->fully_open()) this_open   |= (1 << index);
    index++;
  }

  static unsigned int last_demand = 0;
  static unsigned int last_open = 0;
  if (this_demand != last_demand || this_open != last_open)
    report_state_change( this_demand, last_demand, this_open, last_open );

  if (this_demand == 0 && last_demand != 0)
  {
    overrun_ch = &ch1; // safety net - see comment near definition for choice of ch1.

    int index = 0;
    for (auto chan : channels)
    {
      if (last_open & (1 << index))
        overrun_ch = chan;
      index++;
    }

    Serial.print("Overrun valve is ");
    Serial.println(overrun_ch->tag);
  }

  last_demand = this_demand;
  last_open = this_open;


  unsigned int any_demanded_is_open = this_demand & this_open;
  bool boiler_demand = false;

  if (this_demand != 0)
  {
    my_state( State::demand ); // record-keeping

    for (auto chan : channels)
    {
      if (chan->has_demand())
        chan->open();
      else if (!chan->fully_open())
        chan->close();  // never got fully open; just close.
      else if (any_demanded_is_open)
        chan->close();  // at least one demanded valve is fully open (implictly not this one)
    }

    boiler_demand = any_demanded_is_open;
  }
  else if (overrun_counter_ms > 0)
  {
    my_state( State::overrun );
    boiler_demand = false;

    overrun_ch->open();
      // on the overrun, open this specific channel...
    if (overrun_ch->fully_open())
    {
      // ...then once it's open, close all the others.
      for (auto chan : channels)
        if (chan != overrun_ch)
          chan->close();
    }

  }
  else
  {
    my_state( State::idle );
    boiler_demand = false;

    for (auto chan : channels)
      chan->close(); // belt and braces
  }


  static bool last_boiler_demand = false;
  if (boiler_demand != last_boiler_demand)
  {
    Serial.println((boiler_demand) ? "BOILER ON" : "boiler off");
    if (boiler_demand)
      underrun_counter_ms = underrun_time_ms; // delay demand propagation
  }
  last_boiler_demand = boiler_demand;


  if (boiler_demand)
  {
    if (underrun_counter_ms) // waiting to turn on
    {
      digitalWrite( outputs::boiler_pin, LOW );
      pattern( outputs::boiler_led, 0xf0f0 & mid_brightness );
      underrun_counter_ms--;
    }
    else // on
    {
      digitalWrite( outputs::boiler_pin, HIGH );
      digitalWrite( outputs::boiler_led, HIGH );
      overrun_counter_ms = overrun_time_ms;
    }
  }
  else if (overrun_counter_ms > 0) // off but cooling (overrun)
  {
    digitalWrite( outputs::boiler_pin, LOW );
    pattern( outputs::boiler_led, 0x02c0 & min_brightness ); // 0x02c0 rather than 0xb000 just to desynchronise a little
    overrun_counter_ms--;
  }
  else // off
  {
    digitalWrite( outputs::boiler_pin, LOW );
    digitalWrite( outputs::boiler_led, LOW );
  }
}

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

    valve_hw_led  = A0,
    valve_ch1_led = A1,
    valve_ch2_led = A2,
    valve_ch3_led = A3,
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
  Channel( int (*demand_in_fn)(void*), void (*valve_out_fn)(void*, int), void* fn_context )
    : input_fn{ demand_in_fn }
    , output_fn{ valve_out_fn }
    , fn_context{ fn_context }
    , demand{ false }
    , demand_count{ 0 }
    , curr_state{ Closed }
    , open_close_count{ 0 }
  { }

  enum valve_state { Closed, Opening, Open, Closing };
  valve_state state() const { return curr_state; }

  void ms_poll()
  {
    const int in = input_fn( fn_context );

    if (in && demand_count < demand_count_max)
      demand_count++;     // unequal debounce with 512 steps up...
    else if (!in && demand_count > 0)
      demand_count >>= 1; // ...9 steps down

    if (demand_count == demand_count_max)
      demand = true;
    else if (demand_count == 0)
      demand = false;


    if (curr_state == Opening && open_close_count == 0)
      curr_state = Open;
    else if (curr_state == Closing && open_close_count  == 0)
      curr_state = Closed;

    if (open_close_count > 0)
      --open_close_count;
  }

  void open()
  {
    output_fn( fn_context, HIGH );

    if (curr_state != Open && curr_state != Opening)
    {
      curr_state = Opening;
      open_close_count = open_time_ms;
    }
  }

  void close()
  {
    output_fn( fn_context, LOW );

    if (curr_state != Closed && curr_state != Closing)
    {
      curr_state = Closing;
      open_close_count = close_time_ms;
    }
  }

  bool has_demand() const { return demand; }
  bool fully_open() const { return curr_state == Open; }

private:
  int (*input_fn)(void*);
  void (*output_fn)(void*, int);
  void* fn_context;
    // can't use std::function on AVR Arduino targets.

  bool demand;
  long demand_count;
  enum { demand_count_max = 512 }; // magic number; half a second demand -> on; 9 ms no demand -> off

  valve_state curr_state;
  long open_close_count;
};

class Boiler
{
public:
  Boiler( void (*demand_out_fn)(void*, int), void* fn_context )
    : output_fn{ demand_out_fn }
    , fn_context{ fn_context }
    , curr_demand{ false }
    , curr_state{ Idle }
    , underrun_counter{ 0 }
    , overrun_counter{0 }
    { }

  enum demand_state { Idle, Underrun, Demand, Overrun };
  demand_state state() const { return curr_state; }
  bool idle() const { return curr_state == Idle; }

  void demand( bool demand )
  {
    if (demand != curr_demand)
    {
      Serial.println( (demand) ? "BOILER ON" : "boiler off" );
      if (demand)
        underrun_counter = underrun_time_ms; // delay demand propagation
    }
    curr_demand = demand;
  }

  void ms_poll()
  {
    if (curr_demand)
    {
      if (underrun_counter) // waiting to turn on
      {
        --underrun_counter;
        curr_state = Underrun;
      }
      else // on
      {
        overrun_counter = overrun_time_ms;
        curr_state = Demand;
      }
    }
    else if (overrun_counter > 0) // off but cooling (overrun)
    {
      --overrun_counter;
      curr_state = Overrun;
    }
    else // off
    {
      curr_state = Idle;
    }

    output_fn( fn_context, curr_state == Demand );
  }

private:
  void (*output_fn)(void*, int);
  void* fn_context;
    // can't use std::function on AVR Arduino targets.

  bool curr_demand;
  demand_state curr_state;

  long underrun_counter;
  long overrun_counter;

};




class ChannelEx : public Channel
{
public:
  ChannelEx( int in_pin, int out_pin, int led_pin, const char* tag )
    : Channel( input_fn, output_fn, this )
    , in_pin{ in_pin }
    , out_pin{ out_pin }
    , led_pin{ led_pin }
    , tag{ tag }
  { }
  const int in_pin;
  const int out_pin;
  const int led_pin;
  const char* tag;
private:
  static int input_fn( void* context ) { return digitalRead( ((ChannelEx*)context)->in_pin ); }
  static void output_fn( void* context, int v ) { digitalWrite( ((ChannelEx*)context)->out_pin, v ); }
};

class BoilerEx : public Boiler
{
public:
  BoilerEx( int out_pin, int led_pin )
    : Boiler( output_fn, this )
    , out_pin{ out_pin }
    , led_pin{ led_pin }
  { }
  const int out_pin;
  const int led_pin;
private:
  static void output_fn( void* context, int d ) { digitalWrite( ((BoilerEx*)context)->out_pin, d ); }
};


ChannelEx hw  = { inputs::control_hw_pin, outputs::valve_hw_pin, outputs::valve_hw_led,  "HW"  };
ChannelEx ch1 = { inputs::control_ch1_pin, outputs::valve_ch1_pin, outputs::valve_ch1_led, "CH1" };
ChannelEx ch2 = { inputs::control_ch2_pin, outputs::valve_ch2_pin, outputs::valve_ch2_led, "CH2" };
ChannelEx ch3 = { inputs::control_ch3_pin, outputs::valve_ch3_pin, outputs::valve_ch3_led, "CH3" };
ChannelEx *channels[] = { &hw, &ch1, &ch2, &ch3 };
ChannelEx *overrun_ch = &ch1; // the valve that gets opened on the overrun

// ideally, ch3 would be the default choice, since that'll be the towel rads
// but it's likely this'll be in use before that's plumbed, so needs to work
// safely with only one heating valve: ch1.

BoilerEx boiler = { outputs::boiler_pin, outputs::boiler_led };

// ----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Boiler valve manager");

  pinMode( outputs::status_led, OUTPUT );

  pinMode( inputs::control_hw_pin, INPUT );
  pinMode( outputs::valve_hw_pin, OUTPUT );
  pinMode( outputs::valve_hw_led, OUTPUT );

  pinMode( inputs::control_ch1_pin, INPUT );
  pinMode(outputs::valve_ch1_pin, OUTPUT );
  pinMode(outputs::valve_ch1_led, OUTPUT );

  pinMode( inputs::control_ch2_pin, INPUT );
  pinMode(outputs::valve_ch2_pin, OUTPUT );
  pinMode(outputs::valve_ch2_led, OUTPUT );

  pinMode( inputs::control_ch3_pin, INPUT );
  pinMode(outputs::valve_ch3_pin, OUTPUT );
  pinMode(outputs::valve_ch3_led, OUTPUT );

  pinMode( outputs::boiler_pin, OUTPUT );
  pinMode( outputs::boiler_led, OUTPUT );

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
    chan->ms_poll();
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

    boiler.demand( any_demanded_is_open );
  }
  else if (!boiler.idle()) // ie overrun
  {
    my_state( State::overrun );
    boiler.demand( false );

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
    boiler.demand( false );

    for (auto chan : channels)
      chan->close(); // belt and braces
  }


  boiler.ms_poll();




  for (auto chan : channels)
  {
    switch (chan->state())
    {
      case Channel::Closed:
        digitalWrite( chan->led_pin, LOW );
        break;
      case Channel::Opening:
        pattern( chan->led_pin, 0xff00 & mid_brightness );
        break;
      case Channel::Open:
        digitalWrite( chan->led_pin, HIGH );
        break;
      case Channel::Closing:
        pattern( chan->led_pin, 0x3333 & mid_brightness );
        break;
      default:
        break;
    }
  }

  switch (boiler.state())
  {
    case Boiler::Idle:
      digitalWrite( boiler.led_pin, LOW );
      break;
    case Boiler::Underrun:
      pattern( boiler.led_pin, 0xf0f0 & mid_brightness );
      break;
    case Boiler::Demand:
      digitalWrite( boiler.led_pin, HIGH );
      break;
    case Boiler::Overrun:
      pattern( boiler.led_pin, 0x02c0 & min_brightness ); // 0x02c0 rather than 0xb000 just to desynchronise a little
      break;
    default:
      break;
  }
}

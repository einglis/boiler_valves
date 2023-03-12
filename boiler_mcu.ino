
namespace outputs {
  enum {
    status_pin = LED_BUILTIN, // (aka 13 on Arduino Nano)
    valve_hw_pin  =  8,
    valve_ch1_pin =  9,
    valve_ch2_pin = 10,
    valve_ch3_pin = 11,
    boiler_pin    = 12,
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

// ----------------------------------------------------------------------------

class Channel
{
public:
  Channel( int input_pin, int output_pin )
    : in_pin{ input_pin }
    , out_pin{ output_pin }
    , demand_count{ 0 }
    , demand{ false }
    , open_count{ 0 }
  {}

  void setup()
  {
    pinMode( out_pin, OUTPUT );
    pinMode( in_pin, INPUT );
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

    if (open_count > 0 && open_count < open_count_max)
      open_count++;
  }

  void open()
  {
    digitalWrite( out_pin, true );
    if (open_count == 0)
      open_count = 1;  // get the ball rolling
  }
  void close()
  {
    digitalWrite( out_pin, false );
    open_count = 0;
  }

  bool has_demand() { return demand; }
  bool is_open() { return open_count == open_count_max; }

private:
  const int in_pin;
  const int out_pin;

  int demand_count;
  enum { demand_count_max = 512 };
  bool demand;

  int open_count;
  enum { open_count_max = 2000 };
};

Channel hw  = { inputs::control_hw_pin,  outputs::valve_hw_pin  };
Channel ch1 = { inputs::control_ch1_pin, outputs::valve_ch1_pin };
Channel ch2 = { inputs::control_ch2_pin, outputs::valve_ch2_pin };
Channel ch3 = { inputs::control_ch3_pin, outputs::valve_ch3_pin };
Channel *channels[] = { &hw, &ch1, &ch2, &ch3 };
Channel *overrun_ch = &ch3; // the valve that gets opened on the overrun

// ----------------------------------------------------------------------------

uint32_t status_pattern = 0x00000001;
uint32_t pattern_mask = 0x00000001; // not strictly a mask

enum State { idle, demand, overrun };
State state = State::idle;
int overrun_counter_ms = 0;

// ------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("");

  pinMode( outputs::status_pin, OUTPUT );
  pinMode( outputs::boiler_pin, OUTPUT );

  for (auto chan : channels)
    chan->setup();

  // XXXEDD: watchdog?
}

// ----------------------------------------------------------------------------

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

// ------------------------------------

void loop()
{
  static unsigned long last = millis();
  unsigned long now = millis();
  if (now == last)
    return;
  last = now;


  unsigned int this_demand = 0;
  unsigned int this_open = 0;
  int index = 0;

  for (auto chan : channels)
  {
    chan->poll();
    if (chan->has_demand()) this_demand |= (1 << index);
    if (chan->is_open())    this_open   |= (1 << index);
    index++;
  }

  static unsigned int last_demand = 0;
  static unsigned int last_open = 0;
  if (this_demand != last_demand || this_open != last_open)
    report_state_change( this_demand, last_demand, this_open, last_open );
  last_demand = this_demand;
  last_open = this_open;


  unsigned int any_demanded_is_open = this_demand & this_open;
  bool boiler_demand = false;

  if (this_demand != 0)
  {
    if (state != State::demand)
      Serial.println("NEW STATE: demand");

    state = State::demand;
    status_pattern = 0x00010001; // slow-ish blink

    for (auto chan : channels)
    {
      if (chan->has_demand())
        chan->open();
      else if (!chan->is_open())
        chan->close();  // never got fully open; just close.
      else if (any_demanded_is_open)
        chan->close();  // at least one demanded valve is fully open (implictly not this one)
    }

    boiler_demand = any_demanded_is_open;
  }
  else if (overrun_counter_ms > 0)
  {
    if (state != State::overrun)
      Serial.println("NEW STATE: overrun");

    state = State::overrun;
    status_pattern = 0x11111111; // fast-ish blink

    overrun_ch->open();
      // on the overrun, open this specific channel...
    if (overrun_ch->is_open())
    {
      // ...then once it's open, close all the others.
      for (auto chan : channels)
        if (chan != overrun_ch)
          chan->close();
    }

    boiler_demand = false;
  }
  else
  {
    if (state != State::idle)
      Serial.println("NEW STATE: idle");

    state = State::idle;
    status_pattern = 0x00000001; // slow blink

    for (auto chan : channels)
      chan->close(); // belt and braces

    boiler_demand = false;
  }


  static bool last_boiler_demand = false;
  if (boiler_demand != last_boiler_demand)
    Serial.println((boiler_demand) ? "BOILER ON" : "boiler off");
  last_boiler_demand = boiler_demand;

  digitalWrite( outputs::boiler_pin, boiler_demand);

  if (boiler_demand)
    overrun_counter_ms = 10 * 1000; // 10 seconds for now
  else if (overrun_counter_ms > 0)
    overrun_counter_ms--;


  static long pattern_time = now;
  if (now - pattern_time > 100)
  {
    pattern_mask = (pattern_mask >> 1) | (pattern_mask << 31); // roll right 1
      // roll the mask, then AND with the pattern, to keep nice transitions
    digitalWrite( outputs::status_pin, !!(status_pattern & pattern_mask) );
    pattern_time = now;
  }
}

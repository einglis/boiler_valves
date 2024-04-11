#pragma once

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

  enum {
    valve_open_time_ms = 8000, // seems tremendously slow, but the valves are actally even slower to open fully
    valve_close_time_ms = 4000,
  };

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
      open_close_count = valve_open_time_ms;
    }
  }

  void close()
  {
    output_fn( fn_context, LOW );

    if (curr_state != Closed && curr_state != Closing)
    {
      curr_state = Closing;
      open_close_count = valve_close_time_ms;
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

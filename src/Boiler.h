#pragma once

class Boiler
{
public:
  Boiler( void (*demand_out_fn)(void*, int), void* fn_context )
    : output_fn{ demand_out_fn }
    , fn_context{ fn_context }
    , curr_demand{ false }
    , curr_state{ Idle }
    , overrun_time_ms{ 5 * 60 * 1000L } // five minutes
    , overrun_counter{0 }
    , underrun_time_ms{ 5 * 1000L } // five seconds
    , underrun_counter{ 0 }
    { }

  enum demand_state { Idle, Underrun, Demand, Overrun };
  demand_state state() const { return curr_state; }
  bool is_idle() const { return curr_state == Idle; }

  void demand( bool demand )
  {
    if (demand && !curr_demand)
      underrun_counter = underrun_time_ms; // delay demand propagation
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

  void overrun_time( uint32_t ms ) { overrun_time_ms = ms; }
      // the length of time we hold the overrun valve open once all demand is gone
  void underrun_time( uint32_t ms ) { underrun_time_ms = ms; }
      // the time we wait before allowing the demand signal to propagate to the boiler

private:
  void (*output_fn)(void*, int);
  void* fn_context;
    // can't use std::function on AVR Arduino targets.

  bool curr_demand;
  demand_state curr_state;

   // 16-bits is only a minute of milliseconds, hence we need 32-bits
  uint32_t overrun_time_ms;
  uint32_t overrun_counter;
  uint32_t underrun_time_ms;
  uint32_t underrun_counter;
};

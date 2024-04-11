#pragma once

#include "Boiler.h"
#include "Channel.h"

class Controller
{
public:
  Controller( Channel** channels, int num_channels, Channel* overrun_channel, Boiler* boiler )
    : channels{ channels }
    , num_channels{ num_channels }
    , default_overrun{ overrun_channel }
    , curr_overrun{ overrun_channel }
    , boiler{ boiler }
    , curr_state{ Idle }
    , curr_demand{ 0 }
    , curr_open{ 0 }
  { }

  enum control_state { Idle, Demand, Cool };
  control_state state() const { return curr_state; }
  unsigned int demand() const { return curr_demand; }
  unsigned int open() const { return curr_open; }

  void ms_poll()
  {
    unsigned int this_demand = 0;
    unsigned int this_open = 0;

    for (auto i = 0; i < num_channels; i++)
    {
      if (channels[i]->has_demand()) this_demand |= (1 << i);
      if (channels[i]->fully_open()) this_open |= (1 << i);
    }

    if (this_demand == 0 && curr_demand != 0)
    {
      curr_overrun = default_overrun;

      for (auto i = 0; i < num_channels; i++)
        if (curr_open & (1 << i))
          curr_overrun = channels[i]; // favour later channels
    }

    curr_demand = this_demand;
    curr_open = this_open;


    unsigned int any_demanded_is_open = this_demand & this_open;

    if (this_demand != 0)
    {
      curr_state = Demand;
      boiler->demand( any_demanded_is_open );

      for (auto i = 0; i < num_channels; i++)
      {
        if (channels[i]->has_demand())
          channels[i]->open();
        else if (!channels[i]->fully_open())
          channels[i]->close();  // never got fully open; just close.
        else if (any_demanded_is_open)
          channels[i]->close();  // at least one demanded valve is fully open (implictly not this one)
      }
    }
    else if (!boiler->is_idle()) // ie overrun
    {
      curr_state = Cool;
      boiler->demand( false );

      curr_overrun->open();
        // on the overrun, open this specific channel...
      if (curr_overrun->fully_open())
      {
        // ...then once it's open, close all the others.
        for (auto i = 0; i < num_channels; i++)
          if (channels[i] != curr_overrun)
            channels[i]->close();
      }
    }
    else
    {
      curr_state = Idle;
      boiler->demand( false );

      for (auto i = 0; i < num_channels; i++)
        channels[i]->close(); // belt and braces
    }
  }

private:
  Channel** channels;
  const int num_channels;
  Channel* default_overrun;
  Channel* curr_overrun;
  Boiler* boiler;
  control_state curr_state;
  unsigned int curr_demand;
  unsigned int curr_open;
};

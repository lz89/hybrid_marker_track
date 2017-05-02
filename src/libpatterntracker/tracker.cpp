#include "tracker.h"


/*!
  \file vpTracker.cpp
  \brief Class that defines what is a generic tracker.
*/

void Tracker::init()
{
	cPAvailable = false ;
}



Tracker::Tracker() : cPAvailable(false) {}

Tracker::Tracker(const Tracker &tracker) : cPAvailable(false)
{
	*this = tracker;
}


Tracker &Tracker::operator=(const Tracker &tracker)
{
	cPAvailable = tracker.cPAvailable;

	return *this;
}

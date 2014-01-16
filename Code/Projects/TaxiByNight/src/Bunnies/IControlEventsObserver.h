#ifndef ICONTROL_EVENTS_OBSERVER
#define ICONTROL_EVENTS_OBSERVER

#include <Core/stdint.h>

class Control;

class IControlEventsObserver
{
public:
	virtual ~IControlEventsObserver() {}

	virtual void Clicked(Control *control, uint32_t x, uint32_t y) {};
	virtual void Pressed(Control *control, uint32_t x, uint32_t y) {};
	virtual void Released(Control *control, uint32_t x, uint32_t y) {};
};

#endif // ICONTROL_EVENTS_OBSERVER


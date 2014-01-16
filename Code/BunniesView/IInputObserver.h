#pragma once

#include <windows.h>

class IInputObserver
{
public:
	virtual void KeyDown(int keyCode) = 0;
	virtual void KeyUp(int keyCode) = 0;

	/*virtual void LeftMouseDown() = 0;
	virtual void LeftMouseUp() = 0;
	virtual void RightMouseDown() = 0;
	virtual void RightMouseUp() = 0;*/

	virtual void MouseMove(int x, int y, int xShift, int yShift) = 0;
};

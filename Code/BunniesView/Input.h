#pragma once

#include <windows.h>
#include <vector>

#include "IInputObserver.h"

class Input
{
private:
	std::vector<IInputObserver*> inputObservers;
	bool keyStates[256];
	bool prevKeyStates[256];

	POINT prevMousePos;
	POINT mousePos;
	bool firstMouseMove;

	HWND windowHandle;

	void NotifyKeyDown(BYTE keyCode)
	{
		for (DWORD i = 0; i < inputObservers.size(); i++)
			inputObservers[i] ->KeyDown(keyCode);
	}

	void NotifyKeyUp(BYTE keyCode)
	{
		for (DWORD i = 0; i < inputObservers.size(); i++)
			inputObservers[i] ->KeyUp(keyCode);
	}

	void NotifyMouseMove(int x, int y, int xShift, int yShift)
	{
		for (DWORD i = 0; i < inputObservers.size(); i++)
			inputObservers[i] ->MouseMove(x, y, xShift, yShift);
	}

public:
	Input(HWND windowHandle)
	{
		this ->windowHandle = windowHandle;
	}

	void RegisterObserver(IInputObserver *inputObserver)
	{
		inputObservers.push_back(inputObserver);
	}

	void UpdateMouse()
	{
		GetCursorPosition(mousePos);

		if (firstMouseMove)
		{
			firstMouseMove = false;
			prevMousePos = mousePos;
		}

		if (mousePos.x != prevMousePos.x ||
			mousePos.y != prevMousePos.y)
		{
			NotifyMouseMove(mousePos.x, mousePos.y, mousePos.x - prevMousePos.x, prevMousePos.y - mousePos.y);
			prevMousePos = mousePos;
		}
	}

	void Update()
	{
		UpdateMouse();

		if (windowHandle != GetFocus())
			return;

		SHORT keyState;
		SHORT pressedFlag = (SHORT)0x8000;
		
		for (BYTE i = 0x1; i < 0xfe; i++)
		{
			keyState = GetAsyncKeyState(i);

			keyStates[i] = (keyState & pressedFlag) != 0;

			if (keyStates[i] != prevKeyStates[i])
			{
				prevKeyStates[i] = keyStates[i];
				
				if (keyStates[i])
					NotifyKeyDown(i);
				else
					NotifyKeyUp(i);
			}
		}
	}

	bool GetCursorPosition(POINT &pos)
	{
		GetCursorPos(&pos);
		ScreenToClient(windowHandle, &pos);

		RECT windowRect;
		GetWindowRect(windowHandle, &windowRect);
		windowRect.right -= windowRect.left;
		windowRect.bottom -= windowRect.top;
		windowRect.left = 0;
		windowRect.top = 0;

		return PtInRect(&windowRect, pos);		
	}

	void SetCursorPosition(int x, int y)
	{
		POINT point = {x, y};
		ClientToScreen(windowHandle, &point);

		SetCursorPos(point.x, point.y);
		//prevMousePos = point;
		prevMousePos.x = x;
		prevMousePos.y = y;
	}
};

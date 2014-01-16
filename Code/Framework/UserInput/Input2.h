#pragma once

#include "KeyCode.h"

class Input2
{
public:
	// Should be called only once per loop at the beginning.
	static void UpdateState();
	
	static bool GetKey(KeyCode keyCode);
	static bool GetKeyDown(KeyCode keyCode);
	static bool GetKeyUp(KeyCode keyCode);

private:
	static bool m_pressed[KeyCode::Count];
	static bool m_justPressed[KeyCode::Count];
	static bool m_justReleased[KeyCode::Count];

	static int m_keyMap[KeyCode::Count];

	// user do simulate static constructor
	static bool m_constructorTrigger;

	static bool Constructor();
};

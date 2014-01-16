#include "Input2.h"
#include <Windows.h>

bool Input2::m_pressed[KeyCode::Count];
bool Input2::m_justPressed[KeyCode::Count];
bool Input2::m_justReleased[KeyCode::Count];
int Input2::m_keyMap[KeyCode::Count];

bool Input2::m_constructorTrigger = Input2::Constructor();

bool Input2::Constructor()
{
	Input2::m_keyMap[KeyCode_Left] = VK_LEFT;
	Input2::m_keyMap[KeyCode_Right] = VK_RIGHT;
	Input2::m_keyMap[KeyCode_Up] = VK_UP;
	Input2::m_keyMap[KeyCode_Down] = VK_DOWN;

	Input2::m_keyMap[KeyCode_A] = 'A';
	Input2::m_keyMap[KeyCode_D] = 'D';
	Input2::m_keyMap[KeyCode_S] = 'S';
	Input2::m_keyMap[KeyCode_W] = 'W';
	Input2::m_keyMap[KeyCode_Q] = 'Q';
	Input2::m_keyMap[KeyCode_P] = 'P';

	return true;
}

void Input2::UpdateState()
{
	short keyState;
	short pressedFlag = (short)0x8000;

	for (unsigned char i = 0; i < static_cast<unsigned char>(KeyCode::Count); i++)
	{
		keyState = GetAsyncKeyState(m_keyMap[i]);

		if ((keyState & pressedFlag) != 0)
		{
			m_justPressed[i] = !m_pressed[i];
			
			m_pressed[i] = true;
		}
		else
		{			
			m_justReleased[i] = m_pressed[i];

			m_pressed[i] = false;
		}
	}
}

bool Input2::GetKey(KeyCode keyCode)
{
	return m_pressed[keyCode];
}

bool Input2::GetKeyDown(KeyCode keyCode)
{
	return m_justPressed[keyCode];
}

bool Input2::GetKeyUp(KeyCode keyCode)
{
	return m_justReleased[keyCode];
}


#include "AnimButton.h"
#include <Graphics/SpriteBatch.h>

AnimButton::AnimButton(const std::string &name,
					   float x, float y,
					   TexPart normalState,
					   TexPart pushedState) :
	Control(name)
{
	this->x = x;
	this->y = y;
	this->width = normalState.ImageRect.Width;
	this->height = normalState.ImageRect.Height;
	
	this->normalState = normalState;
	this->bg = normalState;
	this->pushedState = pushedState;
}

void AnimButton::OnDraw(float time, float ms)
{
	sm::Vec2 globalPos = GetGlobalPos();

	TexPart image = m_pressState == PressState_Pressed ? pushedState : normalState;
	
	spriteBatch ->Draw(image, (int)globalPos.x, (int)globalPos.y, width, height);
}

//void AnimButton::OnTouchBegin(int x, int y)
//{
//	this->Control::OnTouchBegin(x, y);
//
//	bg = pushedState;
//	currentShift = pushedShift;
//}
//
//void AnimButton::OnTouchEnd(int x, int y)
//{
//	this->Control::OnTouchEnd(x, y);
//
//	bg = normalState;
//	currentShift = sm::Vec2(0, 0);
//}

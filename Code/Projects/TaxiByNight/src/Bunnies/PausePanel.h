#pragma once

#include "Control.h"
#include "IControlEventsObserver.h"

class GameScreen;

class PausePanel :
	public Control,
	public IControlEventsObserver
{
private:	
	PausePanel(GameScreen *gameScreen);
	
	// Control interface
	void OnDraw(float time, float seconds);

	// IControlEventsObserver interface
	void Clicked(Control *control, uint32_t x, uint32_t y);
	
public:
	static PausePanel *Create(GameScreen *gameScreen);

	GameScreen *m_gameScreen;
};


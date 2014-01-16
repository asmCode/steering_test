#pragma once

#include "Control.h"
#include "IControlEventsObserver.h"

class GameController;

class MainMenuPanel :
	public Control,
	public IControlEventsObserver
{
private:	
	MainMenuPanel(GameController *gameController);
	
	// Control interface
	void OnDraw(float time, float seconds);

	// IControlEventsObserver interface
	void Clicked(Control *control, uint32_t x, uint32_t y);
	
public:
	static MainMenuPanel *Create(GameController *gameController);

	GameController *m_gameController;
};


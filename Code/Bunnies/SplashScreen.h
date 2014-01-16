#pragma once

#include "IScreen.h"
#include <Graphics/Texture.h>
#include "Ticker.h"

class GameController;

class SplashScreen : public IScreen
{
public:
	SplashScreen(GameController *gameController);
	~SplashScreen(void);

	bool InitResources();
	bool ReleaseResources();

	void Draw(float time, float seconds);
	void Update(float time, float seconds);

private:
	GameController *m_gameController;

	Texture *m_splashScreenTex;

	Ticker m_timeout;

	void Leave();
};


#pragma once

#include "IScreen.h"
#include <Graphics/Texture.h>

class GameController;
class MainMenuPanel;
class Label;

class MainMenuScreen : public IScreen
{
public:
	MainMenuScreen(GameController *gameController);
	~MainMenuScreen(void);

	bool InitResources();
	bool ReleaseResources();

	void Draw(float time, float seconds);
	void Update(float time, float seconds);

	void UpdateStats();

private:
	GameController *m_gameController;

	MainMenuPanel *m_mainMenuPanel;

	Label *m_totalIncome;
	Label *m_totalCourses;
	Label *m_bestReward;

	void HandlePress(int pointId, const sm::Vec2 &point);
    void HandleRelease(int pointId, const sm::Vec2 &point);
	void HandleMove(int pointId, const sm::Vec2 &point);
};


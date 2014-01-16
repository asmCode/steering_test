#pragma once

#include "IScreen.h"

class SummaryPanel;
class GameController;

class SummaryScreen : public IScreen
{
public:
	SummaryScreen(GameController *gameController);
	~SummaryScreen(void);

	bool InitResources();
	bool ReleaseResources();

	void Draw(float time, float seconds);
	void Update(float time, float seconds);

	void SetStatistics(
		float earn,
		int courses,
		float totalEarn,
		int totalCourses,
		bool record);

private:
	GameController *m_gameController;

	SummaryPanel *m_summaryPanel;

	void HandlePress(int pointId, const sm::Vec2 &point);
	void HandleRelease(int pointId, const sm::Vec2 &point);
	void HandleMove(int pointId, const sm::Vec2 &point);

	void Enter();
};


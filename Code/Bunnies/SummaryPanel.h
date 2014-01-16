#pragma once

#include "Control.h"
#include "IControlEventsObserver.h"

class GameController;
class Label;
class AnimButton;

class SummaryPanel :
	public Control,
	public IControlEventsObserver
{
public:
	GameController *m_gameController;

	static SummaryPanel *Create(GameController *m_gameController);

	void SetContent(
		float earn,
		int courses,
		float totalEarn,
		int totalCourses,
		bool record);

private:
	Label *m_earnLabel;
	Label *m_coursesLabel;
	Label *m_totalEarnLabel;
	Label *m_totalCoursesLabel;
	Label *m_recordLabel;

	AnimButton *m_mainMenuButton;
	AnimButton *m_againButton;

	bool m_record;

	SummaryPanel();
	
	// Control interface
	void OnDraw(float time, float seconds);

	// IControlEventsObserver interface
	void Clicked(Control *control, uint32_t x, uint32_t y);
};


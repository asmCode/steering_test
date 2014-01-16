#include "SummaryPanel.h"
#include "Environment.h"
#include "InterfaceProvider.h"
#include "AnimButton.h"
#include "Label.h"
#include "SpritesMap.h"
#include "GameController.h"
#include "Taxi.h"
#include "GameController.h"
#include "Inflater.h"
#include <Utils/StringUtils.h>
#include <Graphics/TexPart.h>
#include <Graphics/SpriteBatch.h>
#include <Graphics/Content/Content.h>
#include <Audio/SoundManager.h>
#include <Utils/Log.h>
//#include "MessageBox.h"
//#include "SoundManager.h"

SummaryPanel::SummaryPanel() :
	Control("SummaryPanel")
{
	SetFill(true);
}

SummaryPanel *SummaryPanel::Create(GameController *gameController)
{	
	std::string basePath = TaxiGame::Environment::GetInstance()->GetBasePath();
	SummaryPanel *ret = new SummaryPanel();
	ret->m_gameController = gameController;

	Control *content = Inflater::Inflate(basePath + "data/gui/SummaryPanel.xml");

	ret->AddChild(content);
	
	ret->m_earnLabel = dynamic_cast<Label*>(ret->FindChild("earn_value"));
	assert(ret->m_earnLabel != NULL);
	ret->m_coursesLabel = dynamic_cast<Label*>(ret->FindChild("courses_value"));
	assert(ret->m_coursesLabel != NULL);
	ret->m_totalEarnLabel = dynamic_cast<Label*>(ret->FindChild("total_money_value"));
	assert(ret->m_totalEarnLabel != NULL);
	ret->m_totalCoursesLabel = dynamic_cast<Label*>(ret->FindChild("total_courses_value"));
	assert(ret->m_totalCoursesLabel != NULL);

	ret->m_recordLabel = dynamic_cast<Label*>(ret->FindChild("record"));
	assert(ret->m_recordLabel != NULL);

	ret->m_mainMenuButton = dynamic_cast<AnimButton*>(ret->FindChild("back"));
	assert(ret->m_mainMenuButton != NULL);
	ret->m_againButton = dynamic_cast<AnimButton*>(ret->FindChild("again"));
	assert(ret->m_againButton != NULL);

	ObsCast(IControlEventsObserver, ret->m_mainMenuButton)->AddObserver(ret);
	ObsCast(IControlEventsObserver, ret->m_againButton)->AddObserver(ret);

	return ret;
}

void SummaryPanel::Clicked(Control *control, uint32_t x, uint32_t y)
{
	if (control == m_mainMenuButton)
	{
		SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Button);
		m_gameController->ShowMainMenuScreen();
	}
	else if (control == m_againButton)
	{
		SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Button);
		m_gameController->ShowGameScreen();
	}
}

void SummaryPanel::OnDraw(float time, float seconds)
{
	this->Control::OnDraw(time, seconds);
}

void SummaryPanel::SetContent(
		float earn,
		int courses,
		float totalEarn,
		int totalCourses,
		bool record)
{
	m_earnLabel->SetText(std::string("$") + StringUtils::ToString(earn));
	m_coursesLabel->SetText(StringUtils::ToString(courses));
	m_totalEarnLabel->SetText(std::string("$") + StringUtils::ToString(totalEarn));
	m_totalCoursesLabel->SetText(StringUtils::ToString(totalCourses));
	record ? m_recordLabel->SetVisible(true) : m_recordLabel->SetVisible(false);
}

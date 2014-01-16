#include "HUD.h"
#include "Environment.h"
#include "InterfaceProvider.h"
#include "AnimButton.h"
#include "Label.h"
#include "GameScreen.h"
#include "Taxi.h"
#include "GameController.h"
#include "PedsManager.h"
#include "Inflater.h"
#include <Audio/SoundManager.h>
#include <Graphics/TexPart.h>
#include <Graphics/Content/Content.h>
#include <Utils/Log.h>
//#include "MessageBox.h"
//#include "SoundManager.h"

HUD::HUD() :
	Control("HUD")
{
	SetFill(true);
}

HUD *HUD::Create(GameScreen *gameScreen)
{
	std::string basePath = TaxiGame::Environment::GetInstance()->GetBasePath();
	HUD *ret = new HUD();
	ret->m_gameScreen = gameScreen;

	Control *content = Inflater::Inflate(basePath + "data/gui/HUD.xml");

	ret->AddChild(content);
	
	Control *left = dynamic_cast<AnimButton*>(ret->FindChild("turn_left"));
	assert(left != NULL);
	Control *right = dynamic_cast<AnimButton*>(ret->FindChild("turn_right"));
	assert(right != NULL);
	Control *acc = dynamic_cast<AnimButton*>(ret->FindChild("acc_pedal"));
	assert(acc != NULL);
	Control *brake = dynamic_cast<AnimButton*>(ret->FindChild("brake_pedal"));
	assert(brake != NULL);
	Control *pause = dynamic_cast<AnimButton*>(ret->FindChild("pause"));
	assert(pause != NULL);

	ObsCast(IControlEventsObserver, left)->AddObserver(ret);
	ObsCast(IControlEventsObserver, right)->AddObserver(ret);
	ObsCast(IControlEventsObserver, acc)->AddObserver(ret);
	ObsCast(IControlEventsObserver, brake)->AddObserver(ret);
	ObsCast(IControlEventsObserver, pause)->AddObserver(ret);

	ret->m_totalMoneyLabel = dynamic_cast<Label*>(ret->FindChild("total_money_value"));
	assert(ret->m_totalMoneyLabel != NULL);
	ret->m_totalCoursesLabel = dynamic_cast<Label*>(ret->FindChild("total_courses_value"));
	assert(ret->m_totalCoursesLabel != NULL);
	ret->m_rewardLabel = dynamic_cast<Label*>(ret->FindChild("reward_value"));
	assert(ret->m_rewardLabel != NULL);
	ret->m_timeLeftLabel = dynamic_cast<Label*>(ret->FindChild("time_left_value"));
	assert(ret->m_timeLeftLabel != NULL);
	
	return ret;
}

void HUD::Clicked(Control *control, uint32_t x, uint32_t y)
{
	if (control->GetName() == "pause")
	{
		SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Button);
		m_gameScreen->ShowPause();
	}
}

void HUD::Pressed(Control *control, uint32_t x, uint32_t y)
{
	if (control->GetName() == "turn_left")
		m_gameScreen->TurnLeftButtonPressed(true);
	else if (control->GetName() == "turn_right")
		m_gameScreen->TurnRightButtonPressed(true);
	if (control->GetName() == "acc_pedal")
		m_gameScreen->AccelerationButtonPressed(true);
	if (control->GetName() == "brake_pedal")
		m_gameScreen->BreakButtonPressed(true);
}

void HUD::Released(Control *control, uint32_t x, uint32_t y)
{
	if (control->GetName() == "turn_left")
		m_gameScreen->TurnLeftButtonPressed(false);
	else if (control->GetName() == "turn_right")
		m_gameScreen->TurnRightButtonPressed(false);
	if (control->GetName() == "acc_pedal")
		m_gameScreen->AccelerationButtonPressed(false);
	if (control->GetName() == "brake_pedal")
		m_gameScreen->BreakButtonPressed(false);
}

void HUD::OnDraw(float time, float seconds)
{
	this->Control::OnDraw(time, seconds);
}

void toTime(char *txt, float seconds)
{
	sprintf(txt, "%d:%02d", (int)(seconds / 60.0f), (int)fmodf(seconds, 60.0f));
}

void HUD::OnUpdate(float time, float seconds)
{
	static char txt[128];

	sprintf(txt, "%d", PedsManager::Instance->m_totalCourses);
	m_totalCoursesLabel->SetText(txt);

	sprintf(txt, "$%.2f", PedsManager::Instance->m_totalMoney);
	m_totalMoneyLabel->SetText(txt);

	if (Taxi::GetInstance()->IsOccupied())
	{
		m_rewardLabel->SetVisible(true);
		sprintf(txt, "%.2f", Taxi::GetInstance()->m_revard);
		m_rewardLabel->SetText(txt);

		m_timeLeftLabel->SetVisible(true);
		toTime(txt, Taxi::GetInstance()->m_timeLeft);
		m_timeLeftLabel->SetText(txt);
	}
	else
	{
		m_rewardLabel->SetVisible(false);
		m_timeLeftLabel->SetVisible(false);
	}
}


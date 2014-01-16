#include "PausePanel.h"
#include "Control.h"
#include "Environment.h"
#include "InterfaceProvider.h"
#include "GameScreen.h"
#include "Inflater.h"
#include <Audio/SoundManager.h>
//#include "MessageBox.h"
//#include "SoundManager.h"

PausePanel::PausePanel(GameScreen *gameScreen) :
	Control("PausePanel"),
	m_gameScreen(gameScreen)
{
	SetFill(true);
}

PausePanel *PausePanel::Create(GameScreen *gameScreen)
{	
	PausePanel *ret = new PausePanel(gameScreen);

	std::string basePath = TaxiGame::Environment::GetInstance()->GetBasePath();

	Control *content = Inflater::Inflate(basePath + "data/gui/PausePanel.xml");
	if (content == NULL)
		return NULL;

	Control *endButton = content->FindChild("end");
	assert(endButton != NULL);
	Control *resumeButton = content->FindChild("resume");
	assert(resumeButton != NULL);

	ObsCast(IControlEventsObserver, endButton)->AddObserver(ret);
	ObsCast(IControlEventsObserver, resumeButton)->AddObserver(ret);

	ret->AddChild(content);

	return ret;
}

void PausePanel::Clicked(Control *control, uint32_t x, uint32_t y)
{
	if (control->GetName() == "end")
	{
		SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Button);
		m_gameScreen->EndRound();
	}
	if (control->GetName() == "resume")
	{
		SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Button);
		m_gameScreen->Resume();
	}
}

void PausePanel::OnDraw(float time, float seconds)
{
	this->Control::OnDraw(time, seconds);
}


#include "SummaryScreen.h"
#include "SummaryPanel.h"
#include "InterfaceProvider.h"
#include "GameProps.h"
#include "GameController.h"
#include "Environment.h"
#include <Graphics/Content/Content.h>
#include <Graphics/SpriteBatch.h>
#include <Audio/SoundManager.h>

SummaryScreen::SummaryScreen(GameController *gameController) :
	m_gameController(gameController),
	m_summaryPanel(NULL)
{
}

SummaryScreen::~SummaryScreen(void)
{
}

bool SummaryScreen::InitResources()
{
	m_summaryPanel = SummaryPanel::Create(m_gameController);
	if (m_summaryPanel == NULL)
		return false;

	return true;
}

bool SummaryScreen::ReleaseResources()
{
	if (m_summaryPanel != NULL)
		delete m_summaryPanel;

	return true;
}

void SummaryScreen::Draw(float time, float seconds)
{
	InterfaceProvider::GetSpriteBatch()->Begin();
	m_summaryPanel->Draw(time, seconds);
	InterfaceProvider::GetSpriteBatch()->End();
}

void SummaryScreen::Update(float time, float seconds)
{
	m_summaryPanel->Update(time, seconds);
}

void SummaryScreen::HandlePress(int pointId, const sm::Vec2 &point)
{
	m_summaryPanel->HandlePress(pointId, point);
}

void SummaryScreen::HandleRelease(int pointId, const sm::Vec2 &point)
{
	m_summaryPanel->HandleRelease(pointId, point);
}

void SummaryScreen::HandleMove(int pointId, const sm::Vec2 &point)
{
	m_summaryPanel->HandleMove(pointId, point);
}

void SummaryScreen::SetStatistics(
		float earn,
		int courses,
		float totalEarn,
		int totalCourses,
		bool record)
{
	m_summaryPanel->SetContent(
		earn,
		courses,
		totalEarn,
		totalCourses,
		record);
}

void SummaryScreen::Enter()
{
	SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Summary);
}

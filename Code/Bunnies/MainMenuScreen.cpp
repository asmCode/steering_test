#include "MainMenuScreen.h"
#include "InterfaceProvider.h"
#include "GameProps.h"
#include "GameController.h"
#include "Label.h"
#include "Player.h"
#include "MainMenuPanel.h"
#include <Utils/StringUtils.h>
#include <Graphics/Content/Content.h>
#include <Graphics/SpriteBatch.h>

MainMenuScreen::MainMenuScreen(GameController *gameController) :
	m_gameController(gameController),
	m_mainMenuPanel(NULL)
{
}


MainMenuScreen::~MainMenuScreen(void)
{
}

bool MainMenuScreen::InitResources()
{
	m_mainMenuPanel = MainMenuPanel::Create(m_gameController);

	m_totalIncome = dynamic_cast<Label*>(m_mainMenuPanel->FindChild("total_income_value"));
	assert(m_totalIncome != NULL);
	m_totalCourses = dynamic_cast<Label*>(m_mainMenuPanel->FindChild("total_courses_value"));
	assert(m_totalCourses != NULL);
	m_bestReward = dynamic_cast<Label*>(m_mainMenuPanel->FindChild("best_reward_value"));
	assert(m_bestReward != NULL);

	return true;
}

bool MainMenuScreen::ReleaseResources()
{
	return true;
}

void MainMenuScreen::Draw(float time, float seconds)
{
	InterfaceProvider::GetSpriteBatch()->Begin();
	m_mainMenuPanel->Draw(time, seconds);
	InterfaceProvider::GetSpriteBatch()->End();
}

void MainMenuScreen::Update(float time, float seconds)
{
	m_mainMenuPanel->Update(time, seconds);
}

void MainMenuScreen::HandlePress(int pointId, const sm::Vec2 &point)
{
	m_mainMenuPanel->HandlePress(pointId, point);
}

void MainMenuScreen::HandleRelease(int pointId, const sm::Vec2 &point)
{
	m_mainMenuPanel->HandleRelease(pointId, point);
}

void MainMenuScreen::HandleMove(int pointId, const sm::Vec2 &point)
{
	m_mainMenuPanel->HandleMove(pointId, point);
}

void MainMenuScreen::UpdateStats()
{
	static std::string S = "$";

	m_totalIncome->SetText(S + StringUtils::ToString(Player::Instance->m_totalMoney));
	m_totalCourses->SetText(StringUtils::ToString(Player::Instance->m_totalCourses));
	m_bestReward->SetText(S + StringUtils::ToString(Player::Instance->m_bestRoundIncome));
}

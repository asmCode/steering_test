#include "GameScreen.h"

#include "PedsManager.h"
#include "InterfaceProvider.h"
#include "Environment.h"
#include "GameProps.h"
//#include "ManCam.h"
#include "Street.h"
#include "GameController.h"
#include "DrawingRoutines.h"
#include "PausePanel.h"
#include "Taxi.h"
#include "Arrow.h"
#include "Player.h"
#include "Billboard.h"
#include "PlaceIndicator.h"
#include "Label.h"
#include <Audio/SoundManager.h>
#include "HUD.h"

#include <Math/MathUtils.h>
#include <Graphics/Shader.h>
#include <Graphics/FontRenderer.h>
#include <Graphics/Model.h>
#include <Graphics/Mesh.h>
#include <Graphics/MeshPart.h>
#include <Graphics/Texture.h>
#include <Graphics/SpriteBatch.h>
#include <Graphics/Content/Content.h>
#include <Utils/Log.h>
#include <Utils/StringUtils.h>

#include <Graphics/OpenglPort.h>

GameScreen *GameScreen::m_instance;

GameScreen::GameScreen(GameController *gameController) :
	m_gameController(gameController),
	m_street(NULL),
	m_taxi(NULL),
	m_pedsManager(NULL),
	m_isPaused(false)
{
	m_penaltyProgress = 0.0f;
	m_penaltyTime = 1.6f;
	m_penaltyValue = 0;

	m_fps = 0;
	m_currentFps = 0;
	m_fpsCooldown = 0.0f;

	m_instance = this;

	m_isTurnRightPressed = false;
	m_isTurnLeftPressed = false;

	m_isAccPressed = false;
	m_isBrakePressed = false;
}

GameScreen::~GameScreen(void)
{
}

GameScreen *GameScreen::GetInstance()
{
	return m_instance;
}

bool GameScreen::Initialize()
{
	m_fontKomika = InterfaceProvider::GetFontRenderer("digital_bold_24");

	Billboard::Initialize();

	//m_manCam = new ManCam();

	m_taxi = new Taxi();
	m_pedsManager = new PedsManager(m_taxi->GetPosition());
	m_street = new Street(m_pedsManager);
	m_street->SetInitialVisibility(m_taxi->GetPosition());

	m_arrow = new Arrow();
	m_placeIndicator = new PlaceIndicator();

	m_hud = HUD::Create(this);
	m_pausePanel = PausePanel::Create(this);
	m_pausePanel->Update(0, 0);

	m_messageLabel = dynamic_cast<Label*>(m_hud->FindChild("message"));
	m_penaltyLabel = dynamic_cast<Label*>(m_hud->FindChild("penalty"));

	assert(m_messageLabel != NULL);
	assert(m_penaltyLabel != NULL);

	uint32_t screenWidth = TaxiGame::Environment::GetInstance()->GetScreenWidth();
	uint32_t screenHeight = TaxiGame::Environment::GetInstance()->GetScreenHeight();

	m_projMatrix = sm::Matrix::PerspectiveMatrix(MathUtils::PI / 2.0f, static_cast<float>(screenWidth) / static_cast<float>(screenHeight), 0.1f, 1000.0f);

	return true;
}

bool GameScreen::InitResources()
{
	return false;
}

bool GameScreen::ReleaseResources()
{
	return false;
}

void GameScreen::Draw(float time, float seconds)
{
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	m_street->Draw(time, seconds);
	m_taxi->Draw(time, seconds);
	m_pedsManager->Draw(time, seconds);
	if (!m_isPaused)
		m_arrow->Draw(time, seconds);
	m_placeIndicator->Draw(time, seconds);

	char fpsText[16];
	sprintf(fpsText, "fps: %d", m_currentFps);
	InterfaceProvider::GetSpriteBatch()->Begin();
	m_fontKomika->DrawString(fpsText, 2, TaxiGame::Environment::GetInstance()->GetScreenHeight() - 22, Color::White);
	if (m_isPaused)
		m_pausePanel->Draw(time, seconds);
	else
		m_hud->Draw(time, seconds);
	InterfaceProvider::GetSpriteBatch()->End();

	if (m_penaltyValue != 0.0f)
	{
		m_penaltyLabel->SetVisible(true);

		m_penaltyLabel->SetMarginTop((int)(50.0f + (m_penaltyProgress / m_penaltyTime) * 50.0f));
		m_penaltyLabel->SetColor(Color(255, 0, 0, (uint8_t)((1.0f - m_penaltyProgress / m_penaltyTime) * 255.0f)));
	}
	else
	{
		m_penaltyLabel->SetVisible(false);
	}
}

void GameScreen::SetPenalty(float value)
{
	m_penaltyProgress = 0.0f;
	m_penaltyValue = value;
	m_penaltyLabel->SetText(std::string("$") + StringUtils::ToString(value));
	m_penaltyLabel->SetMarginTop(50);
}

void GameScreen::Update(float time, float seconds)
{
	if (m_penaltyValue != 0.0f)
	{
		m_penaltyProgress += seconds;
		if (m_penaltyProgress > m_penaltyTime)
		{
			m_penaltyProgress = 0.0f;
			m_penaltyValue = 0.0f;
		}
	}

	SimulatePress();

	if (m_isPaused)
	{
		m_pausePanel->Update(time, seconds);
		return;
	}

	m_taxi->Update(time, seconds);

	m_street->SetTaxiPosition(m_taxi->GetPosition());
	m_street->Update(time, seconds);

	m_pedsManager->SetTaxiPosition(m_taxi->GetPosition());
	m_pedsManager->Update(time, seconds);

	if (m_taxi->IsOccupied())
	{
		m_arrow->SetDirection((m_taxi->GetPassengerTarget() - m_taxi->GetPosition()).GetNormalized());
		m_arrow->Update(time, seconds);
	}

	//m_manCam->Process(seconds);

	sm::Vec3 taxiPosition = m_taxi->GetPosition();
	taxiPosition.y = 0.0f;
	sm::Vec3 camPosition = taxiPosition + sm::Vec3(0, 13, -4);
	sm::Vec3 camLook = (camPosition - (taxiPosition + sm::Vec3(0, 0, +2))).GetNormalized();
	m_viewMatrix =
		sm::Matrix::TranslateMatrix(camPosition) *
		sm::Matrix::CreateLookAt2(camLook, sm::Vec3(0, 1, 0));

	m_viewMatrix = m_viewMatrix.GetInversed();

	//m_viewMatrix = m_manCam->GetViewMatrix();
	camPosition = m_viewMatrix.GetInversed() * sm::Vec3(0, 0, 0);

	m_messageLabel->SetVisible(true);
	if (!Player::Instance->m_tutorialFinished && !Taxi::GetInstance()->IsOccupied())
		m_messageLabel->SetText("Stop next to red guy");
	else if (
		!Player::Instance->m_tutorialFinished && Taxi::GetInstance()->IsOccupied() &&
		(Taxi::GetInstance()->GetPassengerTarget() - Taxi::GetInstance()->GetPosition()).GetLength() > 15.0f)
	{
		m_messageLabel->SetText("Follow the red arrow");
	}
	else if (
		!Player::Instance->m_tutorialFinished && Taxi::GetInstance()->IsOccupied() &&
		(Taxi::GetInstance()->GetPassengerTarget() - Taxi::GetInstance()->GetPosition()).GetLength() <= 15.0f)
	{
		m_messageLabel->SetText("Stop next to red area");
	}
	else
		m_messageLabel->SetVisible(false);

	m_placeIndicator->Update(time, seconds, m_projMatrix * m_viewMatrix);
	m_hud->Update(time, seconds);

	m_fps++;
	m_fpsCooldown += seconds;
	if (m_fpsCooldown >= 0.5f)
	{
		m_fpsCooldown = 0.0f;
		m_currentFps = m_fps * 2;
		m_fps = 0;
	}

	DrawingRoutines::SetProjectionMatrix(m_projMatrix);
	DrawingRoutines::SetViewMatrix(m_viewMatrix);
	DrawingRoutines::SetLightPosition(taxiPosition + sm::Vec3(0, 6, 0));
	DrawingRoutines::SetEyePosition(camPosition);

	if (m_taxi->IsOccupied())
	{
		if (m_taxi->m_timeLeft == 0.0f)
		{
			EndRound();
		}
	}
}

void GameScreen::Reset()
{
	m_penaltyProgress = 0.0f;
	m_penaltyTime = 1.6f;
	m_penaltyValue = 0;

	m_pausePanel->SetVisible(false);
	m_isPaused = false;
	SetFreeMode();
	m_taxi->Reset();
	m_pedsManager->Reset(m_taxi->GetPosition());
	m_street->SetInitialVisibility(m_taxi->GetPosition());
}

void GameScreen::SetOccupiedMode()
{
	m_arrow->SetDirection(m_taxi->GetPassengerTarget());
	m_arrow->SetActive(true);
	m_placeIndicator->SetActive(true);
	m_placeIndicator->SetPosition(m_taxi->GetPassengerTarget());
}

void GameScreen::SetFreeMode()
{
	m_arrow->SetActive(false);
	m_placeIndicator->SetActive(false);
}

void GameScreen::HandlePress(int pointId, const sm::Vec2 &point)
{
	m_hud->HandlePress(pointId, point);

	if (m_isPaused)
		m_pausePanel->HandlePress(pointId, point);
}

void GameScreen::HandleRelease(int pointId, const sm::Vec2 &point)
{
	m_hud->HandleRelease(pointId, point);

	if (m_isPaused)
		m_pausePanel->HandleRelease(pointId, point);
}

void GameScreen::HandleMove(int pointId, const sm::Vec2 &point)
{
	m_hud->HandleMove(pointId, point);

	if (m_isPaused)
		m_pausePanel->HandleMove(pointId, point);
}

void GameScreen::TurnLeftButtonPressed(bool isPressed)
{
	m_isTurnLeftPressed = isPressed;

	if (!m_isTurnRightPressed && !m_isTurnLeftPressed)
		m_taxi->SetTurn(0.0f);
	else if (m_isTurnRightPressed)
		m_taxi->SetTurn(-1.0f);
	else if (m_isTurnLeftPressed)
		m_taxi->SetTurn(1.0f);
}

void GameScreen::TurnRightButtonPressed(bool isPressed)
{
	m_isTurnRightPressed = isPressed;

	if (!m_isTurnRightPressed && !m_isTurnLeftPressed)
		m_taxi->SetTurn(0.0f);
	else if (m_isTurnRightPressed)
		m_taxi->SetTurn(-1.0f);
	else if (m_isTurnLeftPressed)
		m_taxi->SetTurn(1.0f);
}

void GameScreen::AccelerationButtonPressed(bool isPressed)
{
	m_isAccPressed = isPressed;

	if (!m_isAccPressed && !m_isBrakePressed)
		m_taxi->SetAcceleration(0.0f);
	else if (m_isAccPressed)
		m_taxi->SetAcceleration(1.0f);
	else if (m_isBrakePressed)
		m_taxi->SetAcceleration(-1.0f);
}

void GameScreen::BreakButtonPressed(bool isPressed)
{
	m_isBrakePressed = isPressed;

	if (!m_isAccPressed && !m_isBrakePressed)
		m_taxi->SetAcceleration(0.0f);
	else if (m_isAccPressed)
		m_taxi->SetAcceleration(1.0f);
	else if (m_isBrakePressed)
		m_taxi->SetAcceleration(-1.0f);
}

void GameScreen::SimulatePress()
{
#if 0
	if (GetAsyncKeyState(VK_UP) & 0x8000)
		AccelerationButtonPressed(true);
	else
		AccelerationButtonPressed(false);
	if (GetAsyncKeyState(VK_DOWN) & 0x8000)
		BreakButtonPressed(true);
	else
		BreakButtonPressed(false);

	if (GetAsyncKeyState(VK_LEFT) & 0x8000)
		TurnLeftButtonPressed(true);
	else
		TurnLeftButtonPressed(false);

	if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
		TurnRightButtonPressed(true);
	else
		TurnRightButtonPressed(false);

#endif
}

void GameScreen::ShowPause()
{
	m_pausePanel->SetVisible(true);
	m_isPaused = true;
}

void GameScreen::Resume()
{
	m_pausePanel->SetVisible(false);
	m_isPaused = false;
}

void GameScreen::EndRound()
{
	bool record = false;
	Player::Instance->m_totalMoney += m_pedsManager->m_totalMoney;
	Player::Instance->m_totalCourses += m_pedsManager->m_totalCourses;
	if (Player::Instance->m_bestRoundIncome < m_pedsManager->m_totalMoney)
	{
		record = true;
		Player::Instance->m_bestRoundIncome = m_pedsManager->m_totalMoney;
	}
	Player::Instance->m_tutorialFinished = true;
	Player::Instance->Save();

	m_gameController->ShowSummaryScreen(
		m_pedsManager->m_totalMoney,
		m_pedsManager->m_totalCourses,
		Player::Instance->m_totalMoney,
		Player::Instance->m_totalCourses,
		record);
}

void GameScreen::Enter()
{
	SoundManager::GetInstance()->StartEngine();
}

void GameScreen::Leave()
{
	SoundManager::GetInstance()->StopEngine();
}

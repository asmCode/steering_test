#include "GameScreen.h"

#include "PedsManager.h"
#include "TrafficManager.h"
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
#include "CarPhysics.h"
#include "VectorGraphics.h"

#include <Math/MathUtils.h>
#include <UserInput/Input2.h>
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

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;
extern std::vector<std::string> debugLog;

GameScreen::GameScreen(GameController *gameController) :
	m_gameController(gameController),
	m_street(NULL),
	m_taxi(NULL),
	m_pedsManager(NULL),
	m_trafficManager(NULL),
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

sm::Matrix view;
sm::Matrix proj;

bool GameScreen::Initialize()
{
	proj = sm::Matrix::Ortho2DMatrix(-20, 20, -20, 20);
	view = sm::Matrix::Identity;

	m_fontKomika = InterfaceProvider::GetFontRenderer("digital_bold_24");
	Content* content = InterfaceProvider::GetContent();

	Billboard::Initialize();

	//m_manCam = new ManCam();

	m_taxi = new Taxi();
	m_pedsManager = new PedsManager(m_taxi->GetPosition());

	m_trafficManager = new TrafficManager();
	m_trafficManager->Initialize();

	m_street = new Street(m_pedsManager, m_trafficManager);
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

	m_spriteShader = content->Get<Shader>("SolidColorSprite");
	assert(m_spriteShader != NULL);
	m_spriteShader->BindVertexChannel(0, "a_position");
	m_spriteShader->LinkProgram();

	m_vectorGraphicsShader = content->Get<Shader>("VectorGraphics");
	assert(m_vectorGraphicsShader != NULL);
	m_vectorGraphicsShader->BindVertexChannel(0, "a_position");
	m_vectorGraphicsShader->LinkProgram();

	VectorGraphics::Initialize(m_vectorGraphicsShader);

	m_carSize.Set(2, 4);
	m_rect1Angle = 0.0f;

	m_carPhysics = new CarPhysics();
	m_carPhysics->SetEngineForce(8.0f * 1000.0f);
	m_carPhysics->SetTotalMass(1000.0f);
	m_carPhysics->SetParameters(-1.8f, 1.8f);

	FontRenderer* font = InterfaceProvider::GetFontRenderer("digital_bold_24");

	int txtLeft = 10;
	int txtTop = 10;
	int txtTopStep = 30;

	/*m_lblSpeed = new Label("", "Speed =", font, 1.0f, Color(255, 255, 255), txtLeft, txtTop);
	txtTop += txtTopStep;
	m_lblLongForce = new Label("", "Long Force =", font, 1.0f, Color(255, 255, 255), txtLeft, txtTop);
	txtTop += txtTopStep;
	m_lblTractionForce = new Label("", "Traction Force =", font, 1.0f, Color(255, 255, 255), txtLeft, txtTop);
	txtTop += txtTopStep;
	m_lblDragForce = new Label("", "Drag Force =", font, 1.0f, Color(255, 255, 255), txtLeft, txtTop);
	txtTop += txtTopStep;
	m_lblResistanceForce = new Label("", "Resistance Force =", font, 1.0f, Color(255, 255, 255), txtLeft, txtTop);*/

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

bool isCollision;

sm::Vec3 MapXZ(const sm::Vec3& v, float y)
{
	return sm::Vec3(v.x, y, v.z);
}

void GameScreen::Draw(float time, float seconds)
{
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);

	sm::Vec4 color1(0.5f, 1.0f, 0.0f, 1.0f);
	sm::Vec4 color2(0.0f, 1.0f, 0.5f, 1.0f);

	if (isCollision)
	{
		color1.Set(1.0f, 0.5f, 0.0f, 1.0f);
		color2.Set(1.0f, 0.0f, 0.5f, 1.0f);
	}

	m_spriteShader->UseProgram();

	sm::Matrix viewMatrix =
		sm::Matrix::TranslateMatrix(0, 0, -10) *
		sm::Matrix::RotateAxisMatrix(MathUtils::PI2, 1, 0, 0);

	VectorGraphics::SetViewProjMatrix(
		proj *
		viewMatrix
		);

	sm::Matrix carTransform = m_carPhysics->GetTransform();

	VectorGraphics::Begin();

	//axis
	VectorGraphics::DrawSegment(
		sm::Vec3(-100.0f, 0, 0),
		sm::Vec3(100.0f, 0, 0));

	VectorGraphics::DrawSegment(
		sm::Vec3(0, 0, -100.0f),
		sm::Vec3(0, 0, 100.0f));
	//

	sm::Vec3 carPosition = carTransform * sm::Vec3(0, 0, 0);

	//VectorGraphics::DrawSegment(m_carPhysics->m_position, m_carPhysics->m_position + m_carPhysics->m_Fe, sm::Vec3(1, 0, 0));
	VectorGraphics::DrawSegment(carPosition, carPosition + m_carPhysics->m_velocity, sm::Vec3(0, 1, 0));

	sm::Vec3 bodyDirection = m_carPhysics->GetBodyDirection();

	VectorGraphics::DrawSegment(carPosition, carPosition + bodyDirection * m_carPhysics->m_velocityLong, sm::Vec3(0, 1, 1));
	VectorGraphics::DrawSegment(carPosition, carPosition + sm::Vec3(bodyDirection.z, 0, -bodyDirection.x) * m_carPhysics->m_velocityLat, sm::Vec3(0, 1, 1));

	sm::Matrix frontAxisTransform =
		carTransform *
		sm::Matrix::TranslateMatrix(0, 0, m_carPhysics->m_frontAxisShift) *
		sm::Matrix::CreateLookAt2(m_carPhysics->GetFrontWheelsLocalDirection().GetReversed(), sm::Vec3(0, 1, 0));

	//axes
	VectorGraphics::DrawSegment(
		frontAxisTransform * sm::Vec3(-10.0f, 0, 0),
		frontAxisTransform * sm::Vec3( 10.0f, 0, 0));

	VectorGraphics::DrawSegment(
		carTransform * sm::Vec3(-10.0f, 0, m_carPhysics->m_rearAxisShift),
		carTransform * sm::Vec3(10.0f, 0, m_carPhysics->m_rearAxisShift));
	//

	// turn radius
	VectorGraphics::DrawSegment(
		carTransform * sm::Vec3(m_carPhysics->CalculateTurnRadius(), 0, m_carPhysics->m_rearAxisShift + 0.2f),
		carTransform * sm::Vec3(m_carPhysics->CalculateTurnRadius(), 0, m_carPhysics->m_rearAxisShift - 0.2f));
	//

	/*
	VectorGraphics::DrawSquare(
		sm::Matrix::TranslateMatrix(m_carPhysics->m_position) *
		sm::Matrix::CreateLookAt2(m_carPhysics->m_bodyDirection.GetReversed(), sm::Vec3(0, 1, 0)) *
		//sm::Matrix::TranslateMatrix(m_carSize.x / 2, 0.0f, -m_carPhysics->m_frontAxisDistance) *
		//sm::Matrix::RotateAxisMatrix(m_carPhysics->m_steerAngle, 0, 1, 0) *
		sm::Matrix::ScaleMatrix(1.0f, 1.0f, 2.0f));
		*/

	VectorGraphics::DrawSquare(
		carTransform *
		//sm::Matrix::TranslateMatrix(m_carSize.x / 2, 0.0f, -m_carPhysics->m_frontAxisDistance) *
		//sm::Matrix::RotateAxisMatrix(m_carPhysics->m_steerAngle, 0, 1, 0) *
		sm::Matrix::ScaleMatrix(2.0f, 1.0f, 4.0f));

	VectorGraphics::End();
}

void GameScreen::SetPenalty(float value)
{
	m_penaltyProgress = 0.0f;
	m_penaltyValue = value;
	m_penaltyLabel->SetText(std::string("$") + StringUtils::ToString(value));
	m_penaltyLabel->SetMarginTop(50);
}

float wheelAngle = 0.0f;

void GameScreen::Update(float time, float seconds)
{
	/*if (Input2::GetKey(KeyCode::KeyCode_Left))
		m_rect1Pos.x -= 0.1f;
	if (Input2::GetKey(KeyCode::KeyCode_Right))
		m_rect1Pos.x += 0.1f;*/
	if (Input2::GetKey(KeyCode::KeyCode_Up))
		m_carPhysics->PushAccelerationPedal(1.0f);
	else
		m_carPhysics->PushAccelerationPedal(0.0f);

	float steerAngle = m_carPhysics->m_steerAngle;

	//wheelAngle = 0.0f;

	if (Input2::GetKey(KeyCode::KeyCode_Left))
	{
		steerAngle -= 2.0f * seconds;
	}

	if (Input2::GetKey(KeyCode::KeyCode_Right))
	{
		steerAngle += 2.0f * seconds;
	}

	//steerAngle = MathUtils::Clamp(steerAngle, -MathUtils::PI4, MathUtils::PI4);

	m_carPhysics->SetSteerAngle(steerAngle);

	m_carPhysics->Update(seconds);

	char text[1024];
	sprintf(text, "Speed = %.2f km/h", m_carPhysics->m_speed * (3600.0f / 1000.0f));
	debugLog.push_back(text);
	sprintf(text, "Speed = %.2f m/s", m_carPhysics->m_speed);
	debugLog.push_back(text);

	debugLog.push_back(std::string("turn radius = " + StringUtils::ToString(m_carPhysics->CalculateTurnRadius())));
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
#if 1
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

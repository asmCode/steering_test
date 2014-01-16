#include "GameController.h"

#include "IScreen.h"
#include "GameScreen.h"
#include "SplashScreen.h"
#include "MainMenuScreen.h"
#include "SummaryScreen.h"
#include "SpritesMap.h"
#include "Player.h"
#include "Environment.h"
#include "Control.h"
#include "Sprite.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include <Graphics/IGraphicsEngine.h>
#include <Graphics/Content/Content.h>
#include <Graphics/Model.h>
#include <Graphics/Animation.h>
#include <Graphics/SpriteBatch.h>
#include <Graphics/Shader.h>
#include <Graphics/FontRenderer.h>
#include <Audio/SoundManager.h>
#include <UserInput/Input2.h>
#include <Utils/Log.h>
#include <time.h>
#include <stddef.h>
#include <stdlib.h>
#include <assert.h>

// TEMP
#include <Graphics/IDrawable.h>
#include <vector>
std::vector<IDrawable*> debugDrawables;
std::vector<std::string> debugLog;
Model *debugSphere;

std::vector<sm::Vec3> debugSpheres;

GameController::GameController(IGraphicsEngine *graphicsEngine) :
	m_graphicsEngine(graphicsEngine),
	m_splashScreen(NULL),
	m_mainMenuScreen(NULL),
	m_summaryScreen(NULL),
	m_gameScreen(NULL),
	m_activeScreen(NULL)
{
}

GameController::~GameController(void)
{
}

bool GameController::InitializeGraphics(const std::string &basePath)
{
	Log::LogT("Initializing Graphics");

	uint32_t screenWidth = TaxiGame::Environment::GetInstance()->GetScreenWidth();
	uint32_t screenHeight = TaxiGame::Environment::GetInstance()->GetScreenHeight();

	m_content = new Content(m_graphicsEngine);
	InterfaceProvider::m_content = m_content;
	m_content->LoadTextures(basePath + "data/gui/");
	m_content->LoadTextures(basePath + "data/textures/");
	m_content->LoadShaders(basePath + "data/shaders/");
	m_content->LoadModels(basePath + "data/models/");
	m_content->LoadAnimations(basePath + "data/animations/");
	m_content->LoadMaterials(basePath + "data/materials/");
	m_content->CombineResources();

	DrawingRoutines::Initialize();
	DrawingRoutines::SetProjectionMatrix(sm::Matrix::PerspectiveMatrix(45.0f, static_cast<float>(screenWidth) / static_cast<float>(screenHeight), 0.1f, 1000.0f));

	Shader *shader = m_content->Get<Shader>("sprite");

	SpriteBatch *spriteBatch = new SpriteBatch(shader, sm::Matrix::Ortho2DMatrix(0, screenWidth, screenHeight, 0));

	SpritesMap *spritesMap = new SpritesMap();
	if (!spritesMap->LoadFromFile(basePath + "data/gui/SpritesMap.xml", m_content))
		return false;

	InterfaceProvider::m_graphicsEngine = m_graphicsEngine;
	InterfaceProvider::m_spriteBatch = spriteBatch;
	InterfaceProvider::m_spritesMap = spritesMap;
	InterfaceProvider::m_fonts["digital_bold_24"] = FontRenderer::LoadFromFile((basePath + "data/fonts/digital_bold_24.xml").c_str(), spriteBatch);
	InterfaceProvider::m_fonts["fenix_18"] = FontRenderer::LoadFromFile((basePath + "data/fonts/fenix_18.xml").c_str(), spriteBatch);
	InterfaceProvider::m_fonts["fenix_26"] = FontRenderer::LoadFromFile((basePath + "data/fonts/fenix_26.xml").c_str(), spriteBatch);

	debugSphere = m_content->Get<Model>("debug_sphere");
	assert(debugSphere != NULL);

	Shader *spriteShader = m_content->Get<Shader>("Sprite2");
	assert(spriteShader != NULL);
	spriteShader->BindVertexChannel(0, "a_position");
	spriteShader->LinkProgram();

	Sprite::Initialize(spriteShader);

	Control::SetSpriteBatch(spriteBatch);
	
	return true;
}

bool GameController::Initialize(ISystemUtils *systemUtils)
{
	Log::StartLog(true, false, false);

	m_systemUtils = systemUtils;

	srand(time(NULL));

	Player *player = new Player(TaxiGame::Environment::GetInstance()->GetWritePath() + "player.xml");
	player->Load();

	std::string basePath = TaxiGame::Environment::GetInstance()->GetBasePath();

	SoundManager::GetInstance()->Initialize(basePath + "data/audio/");
	SoundManager::GetInstance()->PlayMusic();

	if (!InitializeGraphics(basePath))
	{
		assert(false);
		return false;
	}

	m_gameScreen = new GameScreen(this);
	if (!m_gameScreen->Initialize())
		return false;

	m_splashScreen = new SplashScreen(this);
	if (!m_splashScreen->InitResources())
		return false;

	m_mainMenuScreen = new MainMenuScreen(this);
	if (!m_mainMenuScreen->InitResources())
		return false;

	m_summaryScreen = new SummaryScreen(this);
	if (!m_summaryScreen->InitResources())
		return false;

	m_activeScreen = m_splashScreen;

	return true;
}

void GameController::Draw(float time, float seconds)
{
	assert(m_activeScreen != NULL);

	m_activeScreen->Draw(time, seconds);

	for (int i = 0; i < debugDrawables.size(); i++)
		debugDrawables[i]->Draw(time, seconds);

	glClear(GL_DEPTH_BUFFER_BIT);
	for (int i = 0; i < debugSpheres.size(); i++)
		DrawingRoutines::DrawWithMaterial(debugSphere->m_meshParts, sm::Matrix::TranslateMatrix(debugSpheres[i]) * sm::Matrix::ScaleMatrix(0.2f, 0.2f, 0.2f));
	debugSpheres.clear();

	FontRenderer* font = InterfaceProvider::GetFontRenderer("digital_bold_24");
	SpriteBatch* sb = InterfaceProvider::GetSpriteBatch();

	sb->Begin();
	int yShift = 10;
	int yStep = 30;
	for (int i = 0; i < debugLog.size(); i++)
		font->DrawString(debugLog[i].c_str(), 10, yShift += yStep, Color::White, 1.0f);
	sb->End();
	debugLog.clear();
}

void GameController::Update(float time, float seconds)
{
	Input2::UpdateState();

	assert(m_activeScreen != NULL);

	m_activeScreen->Update(time, seconds);
}

void GameController::HandlePress(int pointId, const sm::Vec2 &point)
{
	assert(m_activeScreen != NULL);

	m_activeScreen->HandlePress(pointId, point);
}

void GameController::HandleRelease(int pointId, const sm::Vec2 &point)
{
	assert(m_activeScreen != NULL);

	m_activeScreen->HandleRelease(pointId, point);
}

void GameController::HandleMove(int pointId, const sm::Vec2 &point)
{
	assert(m_activeScreen != NULL);

	m_activeScreen->HandleMove(pointId, point);
}

void GameController::proto_SetStrafeMove(float value)
{
}

void GameController::proto_SetForwardMove(float value)
{
}

void GameController::proto_SetLookTarget(const sm::Vec3 &lookTarget)
{
}

void GameController::ShowGameScreen()
{
	m_gameScreen->Reset();

	m_activeScreen->Leave();
	m_activeScreen = m_gameScreen;
	m_activeScreen->Enter();
}

void GameController::ShowMainMenuScreen()
{
	m_mainMenuScreen->UpdateStats();

	m_activeScreen->Leave();
	m_activeScreen = m_mainMenuScreen;
	m_activeScreen->Enter();
}

void GameController::ShowSummaryScreen(
	float earn,
	int courses,
	float totalEarn,
	int totalCourses,
	bool record)
{
	m_summaryScreen->SetStatistics(
		earn,
		courses,
		totalEarn,
		totalCourses,
		record);

	m_activeScreen->Leave();
	m_activeScreen = m_summaryScreen;
	m_activeScreen->Enter();
}

bool GameController::proto_IsInGame()
{
	return dynamic_cast<GameScreen*>(m_activeScreen) != NULL;
}

void GameController::HandleEnterForeground()
{
	SoundManager::GetInstance()->PlayMusic();

	if (m_activeScreen == m_gameScreen)
		SoundManager::GetInstance()->StartEngine();
}

void GameController::HandleEnterBackground()
{
	SoundManager::GetInstance()->StopMusic();

	if (m_activeScreen == m_gameScreen)
		SoundManager::GetInstance()->StopEngine();
}

void GameController::HandleBackButton()
{
	if (m_activeScreen == m_mainMenuScreen)
		m_systemUtils->QuitApplication();
	else if (m_activeScreen == m_gameScreen)
		m_gameScreen->ShowPause();
}

void GameController::HandleMenukButton()
{

}


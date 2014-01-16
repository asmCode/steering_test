#include "SplashScreen.h"
#include "InterfaceProvider.h"
#include "GameProps.h"
#include "GameController.h"
#include "Environment.h"
#include <Graphics/Content/Content.h>
#include <Graphics/SpriteBatch.h>

SplashScreen::SplashScreen(GameController *gameController) :
	m_gameController(gameController),
	m_splashScreenTex(NULL),
	m_timeout(GameProps::SplashScreenDuration)
{
}


SplashScreen::~SplashScreen(void)
{
}

bool SplashScreen::InitResources()
{
	m_splashScreenTex = InterfaceProvider::GetContent()->Get<Texture>("SplashScreen");

	return true;
}

bool SplashScreen::ReleaseResources()
{
	return true;
}

void SplashScreen::Draw(float time, float seconds)
{
	if (m_splashScreenTex == NULL)
		return;

	InterfaceProvider::GetSpriteBatch()->Begin();
	InterfaceProvider::GetSpriteBatch()->Draw(m_splashScreenTex, 0, 0,
		TaxiGame::Environment::GetInstance()->GetScreenWidth(),
		TaxiGame::Environment::GetInstance()->GetScreenHeight());
	InterfaceProvider::GetSpriteBatch()->End();
}

void SplashScreen::Update(float time, float seconds)
{
	m_timeout.Progress(seconds);
	if (m_timeout.IsTimeout())
		m_gameController->ShowMainMenuScreen();
}

void SplashScreen::Leave()
{
	if (m_splashScreenTex != NULL)
	{
		delete m_splashScreenTex;
		m_splashScreenTex = NULL;
	}
}

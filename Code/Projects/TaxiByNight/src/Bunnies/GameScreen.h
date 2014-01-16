#pragma once

#include "IScreen.h"
#include <Math/Matrix.h>

class IShapesRenderer;
class Street;
class ManCam;
class Taxi;
class FontRenderer;
class PedsManager;
class Arrow;
class PlaceIndicator;
class HUD;
class GameController;
class PausePanel;
class Label;

class GameScreen : public IScreen
{
public:
	GameScreen(GameController *gameController);
	~GameScreen(void);

	static GameScreen *GetInstance();

	bool Initialize();

	bool InitResources();
	bool ReleaseResources();

	void Draw(float time, float seconds);
	void Update(float time, float seconds);

	void Reset();

	void SetOccupiedMode();
	void SetFreeMode();

	void HandlePress(int pointId, const sm::Vec2 &point);
    void HandleRelease(int pointId, const sm::Vec2 &point);
	void HandleMove(int pointId, const sm::Vec2 &point);

	void TurnLeftButtonPressed(bool isPressed);
	void TurnRightButtonPressed(bool isPressed);
	void AccelerationButtonPressed(bool isPressed);
	void BreakButtonPressed(bool isPressed);

	void ShowPause();
	void Resume();
	void EndRound();

	void Enter();
	void Leave();

	float m_penaltyProgress;
	float m_penaltyTime;
	float m_penaltyValue;

	void SetPenalty(float value);

private:
	static GameScreen *m_instance;


	Label *m_messageLabel;
	Label *m_penaltyLabel;

	PausePanel *m_pausePanel;

	GameController *m_gameController;

	bool m_isPaused;

	bool m_isTurnRightPressed;
	bool m_isTurnLeftPressed;

	bool m_isAccPressed;
	bool m_isBrakePressed;

	Street *m_street;
	Taxi *m_taxi;
	PedsManager *m_pedsManager;
	Arrow *m_arrow;
	PlaceIndicator *m_placeIndicator;
	HUD *m_hud;

	FontRenderer *m_fontKomika;

	sm::Matrix m_viewMatrix;
	sm::Matrix m_projMatrix;

	ManCam *m_manCam;

	int m_fps;
	int m_currentFps;
	float m_fpsCooldown;
	float m_fpsUpdatesPerSecond;

	void SimulatePress();
};


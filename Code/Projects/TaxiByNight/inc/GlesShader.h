
#ifndef _GLES_SHADER_H_
#define _GLES_SHADER_H_

#include "../src/Bunnies/ISystemUtils.h"
#include <new>
#include <FApp.h>
#include <FBase.h>
#include <FSystem.h>
#include <FUi.h>
#include <FGraphics.h>
#include <FGraphicsOpengl2.h>
#include <FGrpFloatMatrix4.h>

class IGameController;

/**
 * [GlesShader] application must inherit from Application class
 * which provides basic features necessary to define an application.
 */
class GlesShader
	: public Tizen::App::Application
	, public Tizen::Base::Runtime::ITimerEventListener
	, public Tizen::Ui::IKeyEventListener
	, public Tizen::Ui::ITouchEventListener
	, public ISystemUtils
{
public:
	/**
	 * [GlesShader] application must have a factory method that creates an instance of itself.
	 */
	static Tizen::App::Application* CreateInstance(void);

public:
	GlesShader();
	~GlesShader();

	virtual bool OnAppInitializing(Tizen::App::AppRegistry& appRegistry);
	virtual bool OnAppTerminating(Tizen::App::AppRegistry& appRegistry, bool forcedTermination = false);
	virtual void OnForeground(void);
	virtual void OnBackground(void);
	virtual void OnLowMemory(void);
	virtual void OnBatteryLevelChanged(Tizen::System::BatteryLevel batteryLevel);
	virtual void OnScreenOn(void);
	virtual void OnScreenOff(void);

	virtual void OnTimerExpired(Tizen::Base::Runtime::Timer& timer);

	virtual void OnKeyPressed(const Tizen::Ui::Control& source, Tizen::Ui::KeyCode keyCode);
	virtual void OnKeyReleased(const Tizen::Ui::Control& source, Tizen::Ui::KeyCode keyCode);
	virtual void OnKeyLongPressed(const Tizen::Ui::Control& source, Tizen::Ui::KeyCode keyCode);

	virtual void OnTouchPressed(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo & touchInfo);
	virtual void OnTouchReleased(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo);
	virtual void OnTouchMoved(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo);
	virtual void OnTouchFocusIn(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo);
	virtual void OnTouchFocusOut(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo);

	virtual void QuitApplication();

	bool Draw(void);

private:
	bool InitEGL(void);
	bool InitGL(void);
	void DestroyGL(void);
	void Update(void);
	void Cleanup(void);

private:
	IGameController *m_game;

	Tizen::Ui::Controls::Form*			__pForm;

	Tizen::Graphics::Opengl::EGLDisplay	__eglDisplay;
	Tizen::Graphics::Opengl::EGLSurface	__eglSurface;
	Tizen::Graphics::Opengl::EGLConfig	__eglConfig;
	Tizen::Graphics::Opengl::EGLContext	__eglContext;
	Tizen::Base::Runtime::Timer*			__pTimer;
	Tizen::Graphics::Opengl::GLint		__idxTimeStamp;
	Tizen::Graphics::Opengl::GLfloat		__timeStamp, __strideTimeStamp;
};

#endif

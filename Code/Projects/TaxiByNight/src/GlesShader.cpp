#include "GlesShader.h"
#include "Bunnies/InfectedBunniesFactory.h"
#include "Bunnies/Environment.h"
#include <Graphics/GraphicsEngineFactory.h>
#include <FIo.h>
#include <math.h>

using namespace Tizen::App;
using namespace Tizen::Base;
using namespace Tizen::Base::Runtime;
using namespace Tizen::Base::Utility;
using namespace Tizen::Base::Collection;
using namespace Tizen::System;
using namespace Tizen::Ui;
using namespace Tizen::Ui::Controls;
using namespace Tizen::Graphics;
using namespace Tizen::Graphics::Opengl;

const int TIME_OUT = 0;

long long lastTicks;
float deltaTime;
float timeSinceStart;

class GlesForm
	: public Tizen::Ui::Controls::Form
{
public:
	GlesForm(GlesShader* pApp)
		: __pApp(pApp)
	{
	}

	virtual ~GlesForm(void)
	{
	}

	virtual result OnDraw(void)
	{
		if (__pApp != null)
		{
			__pApp->Draw();
		}
		return E_SUCCESS;
	}

private:
	GlesShader* __pApp;
};

GlesShader::GlesShader(void)
	: __pForm(null)
	, __eglDisplay(EGL_NO_DISPLAY)
	, __eglSurface(EGL_NO_SURFACE)
	, __eglConfig(null)
	, __eglContext(EGL_NO_CONTEXT)
	, __pTimer(null)
	, __idxTimeStamp(0)
	, __timeStamp(1)
	, __strideTimeStamp(0.02)
{
}

GlesShader::~GlesShader(void)
{
}

void
GlesShader::Cleanup(void)
{
	if (__pTimer != null)
	{
		__pTimer->Cancel();
		delete __pTimer;
		__pTimer = null;
	}

	DestroyGL();
}

Application*
GlesShader::CreateInstance(void)
{
	// Create the instance through the constructor.
	return new (std::nothrow) GlesShader();
}

#include <fstream>
#include <Utils/StringUtils.h>

using namespace Tizen::Io;

bool
GlesShader::OnAppInitializing(AppRegistry& appRegistry)
{
	//Log::StartLog(true, false, false);
	result r = E_SUCCESS;
	std::string dataPath;
	std::string writePath;
	IGraphicsEngine *graphicsEngine;

	Frame* pAppFrame = new (std::nothrow) Frame();
	TryReturn(pAppFrame != null, E_FAILURE, "[GlesShader] Generating a frame failed.");

	r = pAppFrame->Construct();
	TryReturn(!IsFailed(r), E_FAILURE, "[GlesShader] pAppFrame->Construct() failed.");

	this->AddFrame(*pAppFrame);

	__pForm = new (std::nothrow) GlesForm(this);
	TryCatch(__pForm != null, , "[GlesShader] Allocation of GlesForm failed.");

	r = __pForm->Construct(FORM_STYLE_NORMAL);
	TryCatch(!IsFailed(r), delete __pForm, "[GlesShader] __pForm->Construct(FORM_STYLE_NORMAL) failed.");

	__pForm->SetOrientation(ORIENTATION_LANDSCAPE);

	r = GetAppFrame()->GetFrame()->AddControl(__pForm);
	TryCatch(!IsFailed(r), delete __pForm, "[GlesShader] GetAppFrame()->GetFrame()->AddControl(__pForm) failed.");

	__pForm->SetMultipointTouchEnabled(true);
	__pForm->AddKeyEventListener(*this);
	__pForm->AddTouchEventListener(*this);

	TryCatch(InitEGL(), , "[GlesShader] GlesCube::InitEGL() failed.");

	TryCatch(InitGL(), , "[GlesShader] GlesCube::InitGL() failed.");

	Update();

	__pTimer = new (std::nothrow) Timer;
	TryCatch(__pTimer != null, , "[GlesShader] Failed to allocate memory.");

	r = __pTimer->Construct(*this);
	TryCatch(!IsFailed(r), , "[GlesShader] __pTimer->Construct(*this) failed.");

	dataPath = StringUtils::ToNarrow((App::GetInstance()->GetAppResourcePath()).GetPointer());
	writePath = StringUtils::ToNarrow((App::GetInstance()->GetAppDataPath()).GetPointer());

	TaxiGame::Environment::GetInstance()->SetBasePath(dataPath);
	TaxiGame::Environment::GetInstance()->SetWritePath(writePath);

	graphicsEngine = GraphicsEngineFactory::Create();
	m_game = InfectedBunniesFactory::Create(graphicsEngine);
	m_game->Initialize(this);

	SystemTime::GetTicks(lastTicks);

	return true;

CATCH:
	AppLog("[GlesShader] GlesShader::OnAppInitializing eglError : %#x\n"
			"[GlesShader] GlesShader::OnAppInitializing glError : %#x\n"
			"[GlesShader] GlesShader::OnAppInitializing VENDOR : %s\n"
			"[GlesShader] GlesShader::OnAppInitializing GL_RENDERER : %s\n"
			"[GlesShader] GlesShader::OnAppInitializing GL_VERSION : %s\n ",
			static_cast<unsigned int>(eglGetError()),
			static_cast<unsigned int>(glGetError()),
			glGetString(GL_VENDOR),
			glGetString(GL_RENDERER),
			glGetString(GL_VERSION));

	Cleanup();

	return false;
}

bool
GlesShader::OnAppTerminating(AppRegistry& appRegistry, bool forcedTermination)
{
	Cleanup();

	return true;
}

void
GlesShader::OnForeground(void)
{
	if (__pTimer != null)
	{
		__pTimer->Start(TIME_OUT);
	}

	if (m_game != NULL)
		m_game->HandleEnterForeground();
}

void
GlesShader::OnBackground(void)
{
	if (__pTimer != null)
	{
		__pTimer->Cancel();
	}

	if (m_game != NULL)
		m_game->HandleEnterBackground();
}

void
GlesShader::OnLowMemory(void)
{
}

void
GlesShader::OnBatteryLevelChanged(BatteryLevel batteryLevel)
{
}

void
GlesShader::OnScreenOn(void)
{
}

void
GlesShader::OnScreenOff(void)
{
}

void
GlesShader::OnTimerExpired(Timer& timer)
{
	if (__pTimer == null)
	{
		return;
	}
	__pTimer->Start(TIME_OUT);

	long long ticks;
	SystemTime::GetTicks(ticks);

	long long deltaTicks = ticks - lastTicks;
	lastTicks = ticks;

	deltaTime = (float)deltaTicks / 1000.0f;

	timeSinceStart += deltaTime;

	if (m_game != NULL)
		m_game->Update(timeSinceStart, deltaTime);

	Update();

	if (!Draw())
	{
		AppLog("[MeshLoader] MeshLoader::Draw() failed.");
	}
}

void
GlesShader::OnKeyPressed(const Control& source, Tizen::Ui::KeyCode keyCode)
{
}

void
GlesShader::OnKeyReleased(const Control& source, Tizen::Ui::KeyCode keyCode)
{
	if (keyCode == Tizen::Ui::KEY_BACK || keyCode == Tizen::Ui::KEY_ESC)
	{
		if (m_game != NULL)
			m_game->HandleBackButton();
	}
}

void
GlesShader::OnKeyLongPressed(const Control& source, Tizen::Ui::KeyCode keyCode)
{
}

void GlesShader::OnTouchPressed(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo & touchInfo)
{
	if (m_game != NULL)
		m_game->HandlePress(touchInfo.GetPointId(), sm::Vec2(currentPosition.x, currentPosition.y));
}

void GlesShader::OnTouchReleased(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo)
{
	if (m_game != NULL)
		m_game->HandleRelease(touchInfo.GetPointId(), sm::Vec2(currentPosition.x, currentPosition.y));
}

void GlesShader::OnTouchMoved(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo)
{
	if (m_game != NULL)
		m_game->HandleMove(touchInfo.GetPointId(), sm::Vec2(currentPosition.x, currentPosition.y));
}

void GlesShader::OnTouchFocusIn(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo)
{
}

void GlesShader::OnTouchFocusOut(const Tizen::Ui::Control& source, const Tizen::Graphics::Point& currentPosition, const Tizen::Ui::TouchEventInfo& touchInfo)
{
}

void
GlesShader::DestroyGL(void)
{
	if (__eglDisplay)
	{
		eglMakeCurrent(__eglDisplay, null, null, null);

		if (__eglContext)
		{
			eglDestroyContext(__eglDisplay, __eglContext);
			__eglContext = EGL_NO_CONTEXT;
		}

		if (__eglSurface)
		{
			eglDestroySurface(__eglDisplay, __eglSurface);
			__eglSurface = EGL_NO_SURFACE;
		}

		eglTerminate(__eglDisplay);
		__eglDisplay = EGL_NO_DISPLAY;
	}

	return;
}

bool
GlesShader::InitEGL(void)
{
	EGLint numConfigs = 1;

	EGLint eglConfigList[] =
	{
		EGL_RED_SIZE,	8,
		EGL_GREEN_SIZE,	8,
		EGL_BLUE_SIZE,	8,
		EGL_ALPHA_SIZE,	0,
		EGL_DEPTH_SIZE, 8,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};

	EGLint eglContextList[] =
	{
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};

	eglBindAPI(EGL_OPENGL_ES_API);

	__eglDisplay = eglGetDisplay((EGLNativeDisplayType)EGL_DEFAULT_DISPLAY);
	TryCatch(__eglDisplay != EGL_NO_DISPLAY, , "[GlesShader] eglGetDisplay() failed.");

	TryCatch(!(eglInitialize(__eglDisplay, null, null) == EGL_FALSE || eglGetError() != EGL_SUCCESS), , "[GlesShader] eglInitialize() failed.");

	TryCatch(!(eglChooseConfig(__eglDisplay, eglConfigList, &__eglConfig, 1, &numConfigs) == EGL_FALSE ||
			eglGetError() != EGL_SUCCESS), , "[GlesShader] eglChooseConfig() failed.");

	TryCatch(numConfigs, , "[GlesShader] eglChooseConfig() failed. because of matching config doesn't exist");

	__eglSurface = eglCreateWindowSurface(__eglDisplay, __eglConfig, (EGLNativeWindowType)__pForm, null);
	TryCatch(!(__eglSurface == EGL_NO_SURFACE || eglGetError() != EGL_SUCCESS), , "[GlesShader] eglCreateWindowSurface() failed.");

	__eglContext = eglCreateContext(__eglDisplay, __eglConfig, EGL_NO_CONTEXT, eglContextList);
	TryCatch(!(__eglContext == EGL_NO_CONTEXT || eglGetError() != EGL_SUCCESS), , "[GlesShader] eglCreateContext() failed.");

	TryCatch(!(eglMakeCurrent(__eglDisplay, __eglSurface, __eglSurface, __eglContext) == EGL_FALSE ||
			eglGetError() != EGL_SUCCESS), , "[GlesShader] eglMakeCurrent() failed.");

	return true;

CATCH:
	{
		AppLog("[GlesShader] GlesShader can run on systems which supports OpenGL ES(R) 2.0.");
		AppLog("[GlesShader] When GlesShader does not correctly execute, there are a few reasons.");
		AppLog("[GlesShader]    1. The current device(real-target or emulator) does not support OpenGL ES(R) 2.0.\n"
				" Check the Release Notes.");
		AppLog("[GlesShader]    2. The system running on emulator cannot support OpenGL(R) 2.1 or later.\n"
				" Try with other system.");
		AppLog("[GlesShader]    3. The system running on emulator does not maintain the latest graphics driver.\n"
				" Update the graphics driver.");
	}

	DestroyGL();

	return false;
}

bool
GlesShader::InitGL(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	return true;
}

void
GlesShader::Update(void)
{
	int x, y, width, height;
	__pForm->GetBounds(x, y, width, height);

	TaxiGame::Environment::GetInstance()->SetScreenSize(width, height);
	glViewport(0, 0, width, height);
}

bool
GlesShader::Draw(void)
{
	if (eglMakeCurrent(__eglDisplay, __eglSurface, __eglSurface, __eglContext) == EGL_FALSE ||
			eglGetError() != EGL_SUCCESS)
	{
		AppLog("[GlesShader] eglMakeCurrent() failed.");

		return false;
	}

	int x, y, width, height;
	__pForm->GetBounds(x, y, width, height);

	glViewport(0, 0, width, height);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	m_game->Draw(timeSinceStart, deltaTime);

	eglSwapBuffers(__eglDisplay, __eglSurface);

	return true;
}

void GlesShader::QuitApplication()
{
	Terminate();
}


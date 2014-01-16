#include <windows.h>

#include <GL/glew.h>
#include "OpenglWindow.h"
#include "TimeControl.h"
#include "Renderer.h"
#include <stdio.h>
#include <assert.h>

#if 0
// iPod Touch 4
int ScreenWidth = 960;
int ScreenHeight = 640;
#endif

#if 0
// galaxy S II
int ScreenWidth = 800;
int ScreenHeight = 480;
#endif

#if 0
// galaxy S III
int ScreenWidth = 1280;
int ScreenHeight = 720;
#endif

#if 1
// custom
int ScreenWidth = 1000;
int ScreenHeight = 1000;
#endif

OpenglWindow *glwnd;
Renderer *renderer;

void APIENTRY OpenglDebugCallback(
	  GLenum source,
	  GLenum type,
	  GLuint id,
	  GLenum severity,
	  GLsizei length,
	  const GLchar* message,
	  void* userParam)
 {
	 int houston = 0;
 }

void InitGl()
{
	if (glewInit() != GLEW_OK)
	{
		assert(false);
	}

	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	glDebugMessageCallback(OpenglDebugCallback, NULL);

	glViewport(0, 0, ScreenWidth, ScreenHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(60.0f, 1024.0f / 768, 1.0f, 1000.0f);
	//gluOrtho2D(0, 1024, 760, 0);
	gluOrtho2D(-50, 50, 50, -50);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glShadeModel(GL_SMOOTH);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_NORMALIZE);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	glDisable(GL_LIGHTING);
}

HWND hwnd;

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int)
{
	TimeControl *tc = new TimeControl();

	tc ->Start();
	float time = 0;

	MSG msg;
	BOOL done = false;

	glwnd = new OpenglWindow();
	glwnd ->SetCurrentContext();
	glwnd ->Initialize(NULL, "dupa", ScreenWidth, ScreenHeight, 32, 0, false, true, NULL);

	InitGl();

	renderer = new Renderer(glwnd);
	renderer ->Initialize();

	while (!done)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
			{
				done = true;
			}
			else
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else
		{
			if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
			{
				done = true;
			}
			else
			{
				float seconds = tc ->CheckFrameMS() / 1000.0f;
				time += seconds;

				renderer->Update(time, seconds);
				renderer ->Render(time, seconds);
			}
		}
	}

	return (int)(msg.wParam);
}


#include "Renderer.h"

#include "../Bunnies/InfectedBunniesFactory.h"
#include "../Bunnies/Player.h"
#include "../Bunnies/Environment.h"

#include <windows.h>
#include <gl\gl.h>
#include <gl\glu.h>
#include <time.h>
#include <assert.h>
#include <Graphics/IGraphicsEngine.h>
#include <Graphics/GraphicsEngineFactory.h>

IGameController *gctrl;
IGraphicsEngine *graphics;

extern int ScreenWidth;
extern int ScreenHeight;

bool mouseDown = false;

Renderer::Renderer(OpenglWindow *glwnd)
{
	this ->glwnd = glwnd;
	input = new Input(glwnd ->GetHwnd());
}

void Renderer::Initialize()
{
	char currentDir[MAX_PATH + 1];
	GetCurrentDirectory(MAX_PATH + 1, currentDir);

	srand(time(NULL));

	input ->RegisterObserver(this);

	graphics = GraphicsEngineFactory::Create();

	gctrl = InfectedBunniesFactory::Create(graphics);

	TaxiGame::Environment::GetInstance()->SetScreenSize(ScreenWidth, ScreenHeight);
	TaxiGame::Environment::GetInstance()->SetBasePath(std::string(currentDir) + "/");

	bool success = gctrl->Initialize(NULL);
	assert(success != NULL);
}

void Renderer::KeyDown(int keyCode)
{
	if (keyCode == VK_SPACE)
		scale = 2.0f;

	switch (keyCode)
	{
	case 'W':
		gctrl->proto_SetForwardMove(-1.0f);
		break;

	case 'S':
		gctrl->proto_SetForwardMove(1.0f);
		break;

	case 'A':
		gctrl->proto_SetStrafeMove(-1.0f);
		break;

	case 'D':
		gctrl->proto_SetStrafeMove(1.0f);
		break;

	case VK_LBUTTON:
		{
			mouseDown = true;
			POINT p;
			GetCursorPos(&p);
			ScreenToClient(glwnd->GetHwnd(), &p);
			gctrl->HandlePress(0, sm::Vec2(p.x, p.y));
		}
	}
}

void Renderer::KeyUp(int keyCode)
{
	if (keyCode == VK_SPACE)
		scale = 1.0f;

	switch (keyCode)
	{
	case 'W':
		gctrl->proto_SetForwardMove(0.0f);
		break;

	case 'S':
		gctrl->proto_SetForwardMove(0.0f);
		break;

	case 'A':
		gctrl->proto_SetStrafeMove(0.0f);
		break;

	case 'D':
		gctrl->proto_SetStrafeMove(0.0f);
		break;

	case VK_LBUTTON:
		{
			mouseDown = false;
			POINT p;
			GetCursorPos(&p);
			ScreenToClient(glwnd->GetHwnd(), &p);
			gctrl->HandleRelease(0, sm::Vec2(p.x, p.y));
		}
	}
}

float xAngle;
float yAngle;

void Renderer::MouseMove(int x, int y, int xShift, int yShift)
{
	xAngle += static_cast<float>(xShift) * 0.004f;
	yAngle += static_cast<float>(yShift) * 0.004f;

	if (yAngle > rad(60.0f))
		yAngle = rad(60.0f);
	else if (yAngle < rad(-60.0f))
		yAngle = rad(-60.0f);

	double modelView[16];
	double projMatrix[16];
	int viewport[4];

	glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
	glGetIntegerv(GL_VIEWPORT, viewport);

	double dx, dy, dz;
	
	gluUnProject(x, y, 0.0, modelView, projMatrix, viewport, &dx, &dy, &dz);

	sm::Vec3 trg = sm::Vec3(0, 0, -1);
	trg.RotateX(yAngle);
	trg.RotateY(xAngle);
	trg.Normalize();

	if (mouseDown)
	{
		gctrl->HandleMove(0, sm::Vec2(x, y));
	}
}

bool mouseVisible = true;

void Renderer::Update(float time, float seconds)
{
	//input->UpdateMouse();
	input->Update();

	if (gctrl->proto_IsInGame())
	{
		if (mouseVisible)
		{
			//ShowCursor(false);
			mouseVisible = false;
		}

		//input->SetCursorPosition(ScreenWidth / 2, ScreenHeight / 2);
	}
	else
	{
		if (!mouseVisible)
		{
			//ShowCursor(true);
			mouseVisible = true;
		}
	}

	gctrl->Update(time, seconds);
}

void Renderer::Render(float time, float seconds)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	gctrl->Draw(time, seconds);

	glwnd ->SwapBuffers();
}


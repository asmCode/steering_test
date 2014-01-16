#include "OpenglWindow.h"

LRESULT CALLBACK OpenglWindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_MOUSEACTIVATE:
		SetFocus(hWnd);
		return 0;

	case WM_SIZE:
		return 0;

	case WM_SYSCOMMAND:
		switch (wParam)
		{
		case SC_SCREENSAVE:
		case SC_MONITORPOWER:
			return 0;
		}

	case WM_DESTROY:
		return 0;
	}

	return DefWindowProc(hWnd, uMsg, wParam, lParam);
}

OpenglWindow::OpenglWindow(void)
{
}

OpenglWindow::~OpenglWindow(void)
{
	Release();
}

bool OpenglWindow::Initialize(HWND parentWindow, const char * title, int width, int height,
				int bpp, int freq, bool fullscreen, bool createOwnWindow, void *params)
{
	if (fullscreen)
		createOwnWindow = true;

	this ->windowWidth = width;
	this ->windowHeight = height;
	this ->fullscreen = fullscreen;
	this ->createOwnWindow = createOwnWindow;
	this ->parentWindow = parentWindow;

	DWORD windowStyle = 0;

	HINSTANCE hinstance = GetModuleHandle(NULL);

	if (createOwnWindow)
	{
		WNDCLASS wc;
		wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
		wc.lpfnWndProc = OpenglWindowProc;
		wc.cbClsExtra = 0;
		wc.cbWndExtra = 0;
		wc.hInstance = hinstance;
		wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = NULL;
		wc.lpszMenuName = NULL;
		wc.lpszClassName = "OpenglWindow";

		if (!RegisterClass(&wc))
		{
			MessageBox(NULL, "Falied To Register Window Class",	"ERROR", MB_OK);
			return false;
		}

		//fullscreen ? windowStyle = WS_POPUP : windowStyle = WS_OVERLAPPED;
		fullscreen ? windowStyle = WS_POPUP : windowStyle = WS_POPUP;
		//if (parentWindow != NULL) windowStyle = WS_CHILD;

		hwnd = CreateWindowEx(0, "OpenglWindow", title, WS_CLIPSIBLINGS | WS_CLIPCHILDREN | windowStyle, 10, 10, width, height,
			parentWindow, NULL, hinstance, params);
		if (hwnd == 0)
		{
			MessageBox(NULL, "Window Creation Error.", "ERROR", MB_OK);
			return false;
		}
	}
	else
	{
		hwnd = parentWindow;
	}

	if (fullscreen)
	{
		DEVMODE screen_settings;
		memset(&screen_settings, 0, sizeof(DEVMODE));
		screen_settings.dmSize = sizeof(screen_settings);
		screen_settings.dmPelsWidth = width;
		screen_settings.dmPelsHeight = height;
		screen_settings.dmBitsPerPel = bpp;
		screen_settings.dmDisplayFrequency = freq;
		screen_settings.dmFields = DM_PELSWIDTH | DM_PELSHEIGHT | DM_BITSPERPEL | DM_DISPLAYFREQUENCY;

		if (ChangeDisplaySettings(&screen_settings, CDS_FULLSCREEN) != DISP_CHANGE_SUCCESSFUL)
		{
			MessageBox(NULL, "Unable To Run In Fullscreen!", "ERROR", MB_OK);
			MessageBox(NULL, "Program Will Now Quit", "ERROR", MB_OK);
			return FALSE;
		}

		//ShowCursor(false);
	}

	static PIXELFORMATDESCRIPTOR pfd = 
	{
		sizeof (PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		bpp,
		0, 0, 0, 0, 0, 0,
		0,
		0,
		0,
		0, 0, 0, 0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0, 0, 0
	};

	hdc = GetDC(hwnd);
	if (hdc == 0)
	{
		MessageBox(NULL, "Can`t Create A GL Device Context", "ERROR", MB_OK);
		return false;
	}

	UINT pixel_format;

	pixel_format = ChoosePixelFormat(hdc, &pfd);
	if (pixel_format == 0)
	{
		MessageBox(NULL, "Can`t Find A Suitable PixelFormat", "ERROR", MB_OK);
		return false;
	}

	if (!SetPixelFormat(hdc, pixel_format, &pfd))
	{
		MessageBox(NULL, "Can`t Set The PixelFormat", "ERROR", MB_OK);
		return false;
	}

	hrc = wglCreateContext(hdc);
	if (hrc == 0)
	{
		MessageBox(NULL, "Can`t Create A GL Rendering Context", "ERROR", MB_OK);
		return false;
	}

	if (!wglMakeCurrent(hdc, hrc))
	{
		MessageBox(NULL, "Can`t Activate The GL Rendering Context", "ERROR", MB_OK);
		return false;
	}

	ShowWindow(hwnd, SW_SHOW);
	SetForegroundWindow(hwnd);
	SetFocus(hwnd);

	return true;
}

void OpenglWindow::Release()
{
	if (fullscreen)
	{
		ChangeDisplaySettings(NULL, 0);
		ShowCursor(TRUE);
	}

	if (hrc)
	{
		wglMakeCurrent(NULL, NULL);
		wglDeleteContext(hrc);
		hrc = NULL;
	}

	if (hdc)
	{
		ReleaseDC(hwnd, hdc);
		hdc = NULL;
	}

	if (hwnd && createOwnWindow)
	{
		DestroyWindow(hwnd);
		hwnd = NULL;

		UnregisterClass("OpenglWindow", GetModuleHandle(NULL));
	}
}

void OpenglWindow::Resize(int width, int height)
{
	this ->windowWidth = width;
	this ->windowHeight = height;

	SetWindowPos(hwnd, NULL, 0, 0, width, height, SWP_NOMOVE | SWP_NOOWNERZORDER);
}

int OpenglWindow::GetWindowWidth()
{
	return windowWidth;
}

int OpenglWindow::GetWindowHeight()
{
	return windowHeight;
}

void OpenglWindow::SetCurrentContext()
{
	wglMakeCurrent(hdc, hrc);
}

void OpenglWindow::SwapBuffers()
{
	::SwapBuffers(hdc);
}

HDC OpenglWindow::GetHDC()
{
	return hdc;
}

HWND OpenglWindow::GetHwnd()
{
	return hwnd;
}

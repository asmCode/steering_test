#include "Control.h"
#include "Environment.h"
#include <assert.h>
#include <Graphics/SpriteBatch.h>
#include <Graphics/TexPart.h>
#include <Graphics/Color.h>

SpriteBatch *Control::spriteBatch;

void Control::SetSpriteBatch(SpriteBatch *spriteBatch)
{
	Control::spriteBatch = spriteBatch;
}

Control::Control(const std::string &name) :
	m_name(name),
	m_fill(false)
{
	SetDefaults();

	x = 0;
	y = 0;
	width = 0;
	height = 0;
	opacity = 1.0f;
	
	parent		= NULL;
	visible		= true;
	enabled		= true;
}

Control::Control(const std::string &name, int x, int y, int width, int height) :
	m_name(name),
	m_fill(false)
{
	SetDefaults();

	this ->x = x;
	this ->y = y;
	this ->width = width;
	this ->height = height;
	opacity = 1.0f;
	
	parent		= NULL;
	visible		= true;
	enabled		= true;
}

Control::Control(const std::string &name, int x, int y, const TexPart &bg) :
	m_name(name),
	m_fill(false)
{
	SetDefaults();

	this ->x = x;
	this ->y = y;
	this ->width = bg.ImageRect.Width;
	this ->height = bg.ImageRect.Height;
	opacity = 1.0f;
	
	parent = NULL;
	visible = true;
	enabled		= true;
	this ->bg = bg;
}

Control::Control(const std::string &name, int x, int y, int width, int height, const TexPart &bg) :
	m_name(name),
	m_fill(false)
{
	SetDefaults();

	this ->x = x;
	this ->y = y;
	this ->width = width;
	this ->height = height;
	opacity = 1.0f;
	
	parent = NULL;
	visible = true;
	enabled	= true;
	this ->bg = bg;
}

Control::~Control()
{
	std::map<std::string, unsigned char*>::iterator it;
	for (it = userData.begin(); it != userData.end(); it++)
		delete [] it ->second;
}

void Control::SetDefaults()
{
	x = 0;
	y = 0;
	width = 0;
	height = 0;

	m_marginLeft = 0;
	m_marginRight = 0;
	m_marginTop = 0;
	m_marginBottom = 0;

	m_tmpFill = "";

	m_pressState = PressState_Unpressed;
	m_pressedFingerId = 0;
}

Control *Control::GetParent()
{
	return parent;
}

void Control::AddChild(Control *control)
{
	if (control ->GetParent() != NULL)
	{
		assert(false && "Control is child of other parent");
		return;
	}

	control ->parent = this;
	children.push_back(control);
}

void Control::SetX(int x)
{
	this->x = x;
}

void Control::SetY(int y)
{
	this->y = y;
}

void Control::SetPosition(int x, int y)
{
	this ->x = x;
	this ->y = y;
}

void Control::SetSize(int width, int height)
{
	this ->width = width;
	this ->height = height;
}

void Control::SetWidth(int width)
{
	this->width = width;
}

void Control::SetHeight(int height)
{
	this->height = height;
}

void Control::SetMarginLeft(int value)
{
	m_marginLeft = value;
}

void Control::SetMarginRight(int value)
{
	m_marginRight = value;
}

void Control::SetMarginTop(int value)
{
	m_marginTop = value;
}

void Control::SetMarginBottom(int value)
{
	m_marginBottom = value;
}

void Control::SetBounds(int x, int y, int width, int height)
{
	SetPosition(x, y);
	SetSize(width, height);
}

void Control::SetFill(bool fill)
{
	m_fill = fill;
}

void Control::SetAlign(const std::string &align)
{
	m_align = align;
}

sm::Vec2 Control::GetLocalPos()
{
	return sm::Vec2(x, y);
}

sm::Vec2 Control::GetGlobalPos()
{
	sm::Vec2 pos(x, y);
	if (parent != NULL)
		pos += parent ->GetGlobalPos();
	return pos;
}

std::string Control::GetName() const
{
	return m_name;
}

//void Control::HandleTapGesture(const sm::Vec2 &point)
//{
//	if (!visible || !enabled)
//		return;
//	
//	std::list<Control*>::iterator it;
//	for (it = children.begin(); it != children.end(); it++)
//		(*it) ->HandleTapGesture(sm::Vec2(point.x - this->x, point.y - this->y));
//	
//	if (HitTest(point.x, point.y))
//	{
//		OnTouch(point.x - this->x, point.y - this->y);
//	}
//}

void Control::HandlePanGesture(IGestureHandler::GestureStatus status,
							   const sm::Vec2 &pos,
							   const sm::Vec2 &trans,
							   const sm::Vec2 &velocity)
{
	if (!visible || !enabled)
		return;

	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
		(*it) ->HandlePanGesture(status, pos, trans, velocity);
}


void Control::HandlePress(int pointId, const sm::Vec2 &point)
{
	if (!visible || !enabled)
		return;

	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
		(*it) ->HandlePress(pointId, sm::Vec2(point.x - this->x, point.y - this->y));
	
	if (HitTest(point.x, point.y))
	{
		if (m_pressState == PressState_Unpressed)
		{
			m_pressState = PressState_Pressed;
			m_pressedFingerId = pointId;

			OnTouchBegin(pointId, point.x - this->x, point.y - this->y);
		}
	}
}

void Control::HandleRelease(int pointId, const sm::Vec2 &point)
{
	if (!visible || !enabled)
		return;

	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
		(*it) ->HandleRelease(pointId, sm::Vec2(point.x - this->x, point.y - this->y));
	
	if (HitTest(point.x, point.y))
	{
		if (m_pressState == PressState_Pressed &&
			m_pressedFingerId == pointId)
		{
			m_pressState = PressState_Unpressed;
			m_pressedFingerId = 0;

			OnTouchEnd(pointId, point.x - this->x, point.y - this->y);
			OnClicked(pointId, point.x - this->x, point.y - this->y);
		}
	}
}

void Control::HandleMove(int pointId, const sm::Vec2 &point)
{
	if (!visible || !enabled)
		return;

	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
		(*it) ->HandleMove(pointId, sm::Vec2(point.x - this->x, point.y - this->y));
	
	if (HitTest(point.x, point.y))
	{
	}
	else
	{
		if (m_pressState == PressState_Pressed &&
			m_pressedFingerId == pointId)
		{
			m_pressState = PressState_Unpressed;
			m_pressedFingerId = 0;

			OnTouchEnd(pointId, point.x - this->x, point.y - this->y);
		}
	}
}

bool Control::HitTest(int x, int y) const
{
	return
		(x >= this ->x && x < this ->x + width) &&
		(y >= this ->y && y < this ->y + height);
}

void Control::Update(float time, float ms)
{
	if (!visible)
		return;

	OnUpdate(time, ms);

	sm::Vec2 parentSize = GetParentSize();

	if (m_tmpFill == "fit")
	{
		float controlAspect = (float)width / (float)height;
		float parentAspect = parentSize.x / parentSize.y;

		if (controlAspect > parentAspect)
		{
			width = parentSize.x;
			height = (int)(float)width / controlAspect;
		}
		else
		{
			height = parentSize.y;
			width = (int)(float)height * controlAspect;
		}
	}
	else if (m_tmpFill == "wypelnij")
	{
		float controlAspect = (float)width / (float)height;
		float parentAspect = parentSize.x / parentSize.y;

		if (controlAspect > parentAspect)
		{
			width = parentSize.x;
			height = (int)(float)width / controlAspect;
		}
		else
		{
			width = parentSize.x;
			height = (int)(float)width / controlAspect;
		}
	}

	if (m_fill)
	{
		x = 0;
		y = 0;
		width = parentSize.x;
		height = parentSize.y;
	}
	else if (!m_fill && m_align =="center")
	{
		x = (parentSize.x - width) / 2;
		y = (parentSize.y - height) / 2;
	}
	else if (!m_fill && m_align =="bottom")
	{
		x = (parentSize.x - width) / 2;
		y = parentSize.y - height - m_marginBottom;
	}
	else if (!m_fill && m_align =="top")
	{
		x = (parentSize.x - width) / 2;
		y = m_marginTop;
	}
	else if (!m_fill && m_align =="left")
	{
		x = m_marginLeft;
		y = (parentSize.y - height) / 2;
	}
	else if (!m_fill && m_align =="right")
	{
		x = parentSize.x - width - m_marginRight;
		y = (parentSize.y - height) / 2;
	}
	else if (!m_fill && m_align =="bottom-left")
	{
		x = m_marginLeft;
		y = parentSize.y - height - m_marginBottom;
	}
	else if (!m_fill && m_align =="bottom-right")
	{
		x = parentSize.x - width - m_marginRight;
		y = parentSize.y - height - m_marginBottom;
	}
	else if (!m_fill && m_align =="top-left")
	{
		x = m_marginLeft;
		y = m_marginTop;
	}
	else if (!m_fill && m_align =="top-right")
	{
		x = parentSize.x - width - m_marginRight;
		y = m_marginTop;
	}
	
	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
		(*it) ->Update(time, ms);
}

void Control::Draw(float time, float ms)
{
	assert(spriteBatch != NULL);

	if (!visible)
		return;
	
	OnDraw(time, ms);
	
	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
		(*it) ->Draw(time, ms);	
}

void Control::OnTouch(int x, int y)
{
	ObsCast(IControlEventsObserver, this)->NotifyObservers(&IControlEventsObserver::Clicked, this, x, y);
}

void Control::OnTouchBegin(int pointId, int x, int y)
{
	ObsCast(IControlEventsObserver, this)->NotifyObservers(&IControlEventsObserver::Pressed, this, x, y);
}

void Control::OnTouchEnd(int pointId, int x, int y)
{
	ObsCast(IControlEventsObserver, this)->NotifyObservers(&IControlEventsObserver::Released, this, x, y);
}

void Control::OnClicked(int pointId, int x, int y)
{
	ObsCast(IControlEventsObserver, this)->NotifyObservers(&IControlEventsObserver::Clicked, this, x, y);
}

void Control::OnTouchMoved(int pointId, int x, int y)
{
	//ObsCast(IControlEventsObserver, this)->NotifyObservers(&IControlEventsObserver::Released, this, x, y);
	//ObsCast(IControlEventsObserver, this)->NotifyObservers(&IControlEventsObserver::Clicked, this, x, y);
}

void Control::OnDraw(float time, float ms)
{
	if (bg.Tex != NULL)
	{
		sm::Vec2 globalPos = GetGlobalPos();
		spriteBatch ->Draw(bg,
						   Color(255, 255, 255, (unsigned char)(255.0f * opacity)),
						   (int)globalPos.x,
						   (int)globalPos.y,
						   width,
						   height);
	}
}

void Control::OnUpdate(float time, float ms)
{
}

int Control::GetX() const
{
	return x;
}

int Control::GetY() const
{
	return y;
}

int Control::GetWidth() const
{
	return width;
}

int Control::GetHeight() const
{
	return height;
}

void Control::SetVisible(bool visible)
{
	this->visible = visible;
}

void Control::SetEnabled(bool enabled)
{
	this->enabled = enabled;
}

bool Control::IsVisible() const
{
	return visible;
}

bool Control::IsEnabled() const
{
	return enabled;
}

sm::Vec2 Control::GetParentSize() const
{
	static int screenWidth = TaxiGame::Environment::GetInstance()->GetScreenWidth();
	static int screenHeight = TaxiGame::Environment::GetInstance()->GetScreenHeight();

	if (parent != NULL)
		return sm::Vec2(parent->GetWidth(), parent->GetHeight());
	else
		return sm::Vec2(screenWidth, screenHeight);
}

float Control::GetOpacity() const
{
	return opacity;
}

void Control::SetOpacity(float opacity)
{
	this->opacity = opacity;
}

Control* Control::FindChild(const std::string &name)
{
	if (GetName() == name)
		return this;

	std::list<Control*>::iterator it;
	for (it = children.begin(); it != children.end(); it++)
	{
		Control *child = (*it)->FindChild(name);
		if (child != NULL)
			return child;
	}

	return NULL;
}


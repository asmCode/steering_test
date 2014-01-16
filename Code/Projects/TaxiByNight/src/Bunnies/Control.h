#pragma once

#include <list>
#include <map>
#include <string>
#include <Math/Vec2.h>
#include <Graphics/IDrawable.h>
#include <Graphics/IUpdateable.h>
#include <Graphics/Point.h>
#include <Core/ObserverSubject.h>
#include "IControlEventsObserver.h"
#include <UI/IGestureHandler.h>
#include <Graphics/TexPart.h>

class SpriteBatch;

class Control :
	public virtual IUpdateable,
	public virtual IDrawable,
	public virtual IGestureHandler,
	public ObserversContainer<IControlEventsObserver*>
{
protected:
	enum PressState
	{
		PressState_Unpressed,
		PressState_Pressed
	};

	int x;
	int y;
	int width;
	int height;
	int m_marginLeft;
	int m_marginRight;
	int m_marginTop;
	int m_marginBottom;

	PressState m_pressState;
	int m_pressedFingerId;
	
	float opacity;
	
	bool visible;
	bool enabled;
	
	Control *parent;
	std::list<Control*> children;
	mutable std::map<std::string, unsigned char*> userData;
	
	TexPart bg;
	static SpriteBatch *spriteBatch;

	std::string m_name;

	bool m_fill;
	std::string m_align;

	void SetDefaults();
	
	virtual void OnTouch(int x, int y);
	virtual void OnTouchBegin(int pointId, int x, int y);
	virtual void OnTouchEnd(int pointId, int x, int y);
	virtual void OnClicked(int pointId, int x, int y);
	virtual void OnTouchMoved(int pointId, int x, int y);
	virtual void OnDraw(float time, float ms);
	virtual void OnUpdate(float time, float ms);
	
public:
	std::string m_tmpFill;

	static void SetSpriteBatch(SpriteBatch *spriteBatch);
	
	Control(const std::string &name);
	Control(const std::string &name, int x, int y, int width, int height);
	Control(const std::string &name, int x, int y, const TexPart &bg);
	Control(const std::string &name, int x, int y, int width, int height, const TexPart &bg);
	virtual ~Control();
	
	Control *GetParent();
	void AddChild(Control *control);
	
	void SetX(int x);
	void SetY(int y);
	void SetPosition(int x, int y);
	void SetSize(int width, int height);
	void SetWidth(int width);
	void SetHeight(int height);

	void SetMarginLeft(int value);
	void SetMarginRight(int value);
	void SetMarginTop(int value);
	void SetMarginBottom(int value);

	void SetBounds(int x, int y, int width, int height);
	void SetFill(bool fill);
	void SetAlign(const std::string &align);
	void SetVisible(bool visible);
	void SetEnabled(bool enabled);
	
	int GetX() const;
	int GetY() const;
	int GetWidth() const;
	int GetHeight() const;
	bool IsVisible() const;
	bool IsEnabled() const;

	sm::Vec2 GetParentSize() const;

	Control* FindChild(const std::string &name);

	std::string GetName() const;
	
	float GetOpacity() const;
	void SetOpacity(float opacity);
	
	sm::Vec2 GetLocalPos();
	sm::Vec2 GetGlobalPos();
	
	void HandlePanGesture(IGestureHandler::GestureStatus status,
						  const sm::Vec2 &pos,
						  const sm::Vec2 &trans,
						  const sm::Vec2 &velocity);
	//void HandleTapGesture(const sm::Vec2 &point);
	void HandlePress(int pointIndex, const sm::Vec2 &point);
	void HandleRelease(int pointIndex, const sm::Vec2 &point);
	void HandleMove(int pointId, const sm::Vec2 &point);

	virtual bool HitTest(int x, int y) const;
	
	virtual void Update(float time, float ms);
	virtual void Draw(float time, float ms);
	
	template <typename T>
	void SetUserData(const std::string &name, const T &value)
	{
		unsigned char *valCopy = new unsigned char[sizeof(T)];
		memcpy(valCopy, &value, sizeof(T));
		userData[name] = valCopy;
	}
	
	template <typename T>
	bool GetUserData(const std::string &name, T &value) const
	{
		std::map<std::string, unsigned char*>::iterator it = userData.find(name);
		if (it == userData.end())
			return false;
		
		memcpy(&value, it ->second, sizeof(T));
		return true;
	}
};

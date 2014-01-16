#pragma once

#include "Control.h"
#include <Graphics/Color.h>
#include <string>

class FontRenderer;

class Label : public Control
{
private:
	std::string text;
	Color textColor;
	FontRenderer *fontRenderer;
	float m_size;
	
public:
	Label(const std::string &name,
		  const std::string &text,
		  FontRenderer *fontRenderer,
		  float size,
		  const Color &textColor,
		  int x,
		  int y);
	
	void SetText(const std::string &text);
	const std::string& GetText();
	void SetColor(const Color &color);
	
	void OnUpdate(float time, float ms);
	void OnDraw(float time, float ms);
	
	sm::Point<int> GetTextSize();

	void RecalculateSize();
};

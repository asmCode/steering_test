#include "Label.h"
#include <Graphics/FontRenderer.h>

Label::Label(const std::string &name,
			 const std::string &text,
			 FontRenderer *fontRenderer,
			 float size,
			 const Color &textColor,
			 int x,
			 int y) :
	Control(name, x, y, 0, 0),
	m_size(size)
{
	this ->text = text;
	this ->fontRenderer = fontRenderer;
	this ->textColor = textColor;
	this->opacity = 1.0f;

	RecalculateSize();
}

void Label::OnUpdate(float time, float ms)
{
}

void Label::OnDraw(float time, float ms)
{
	Color color = textColor;
	if (parent != NULL)
		color.A = (unsigned char)((float)color.A * parent->GetOpacity() * opacity);
	sm::Vec2 globalPos = GetGlobalPos();
	fontRenderer ->DrawString(text.c_str(), globalPos.x, globalPos.y, color, m_size);
}

void Label::SetText(const std::string &text)
{
	this ->text = text;

	RecalculateSize();
}

const std::string& Label::GetText()
{
	return this ->text;
}

sm::Point<int> Label::GetTextSize()
{
	sm::Point<int> size = fontRenderer->MeasureString(text.c_str());
	size.X = (int)((float)size.X * m_size);
	size.Y = (int)((float)size.Y * m_size);

	return size;
}

void Label::RecalculateSize()
{
	sm::Point<int> textSize = GetTextSize();
	width = textSize.X;
	height = textSize.Y;
}

void Label::SetColor(const Color &color)
{
	textColor = color;
}

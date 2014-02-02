#include "GraphicsLog.h"
#include <Graphics/FontRenderer.h>
#include <Graphics/SpriteBatch.h>

#include "VectorGraphics.h"

SpriteBatch *GraphicsLog::m_spriteBatch;
FontRenderer *GraphicsLog::m_font;

std::vector<GraphicsLog::LogItem> GraphicsLog::m_logs;
std::vector<GraphicsLog::SegmentItem> GraphicsLog::m_segments;
std::vector<GraphicsLog::SquareItem> GraphicsLog::m_squares;

void GraphicsLog::Initialize(SpriteBatch* spriteBatch, FontRenderer* fontRenderer)
{
	m_spriteBatch = spriteBatch;
	m_font = fontRenderer;
}

void GraphicsLog::AddLog(const std::string& log)
{
	LogItem item;
	item.m_log = log;

	m_logs.push_back(item);
}

void GraphicsLog::AddSegment(const sm::Vec3& begin, const sm::Vec3& end, const sm::Vec3& color)
{
	SegmentItem item;
	item.m_begin = begin;
	item.m_end = end;
	item.m_color = color;

	m_segments.push_back(item);
}

void GraphicsLog::DrawSquare(const sm::Matrix& transform, const sm::Vec3& strokeColor)
{
	SquareItem item;
	item.m_transform = transform;
	item.m_strokeColor = strokeColor;

	m_squares.push_back(item);
}

void GraphicsLog::DrawAndClear()
{
	m_spriteBatch->Begin();
	int yShift = 10;
	int yStep = 30;
	for (unsigned int i = 0; i < m_logs.size(); i++)
		m_font->DrawString(m_logs[i].m_log.c_str(), 10, yShift += yStep, Color::White, 1.0f);
	m_spriteBatch->End();

	VectorGraphics::Begin();

	for (unsigned int i = 0; i < m_segments.size(); i++)
		VectorGraphics::DrawSegment(m_segments[i].m_begin, m_segments[i].m_end, m_segments[i].m_color);

	for (unsigned int i = 0; i < m_squares.size(); i++)
		VectorGraphics::DrawSquare(m_squares[i].m_transform, m_squares[i].m_strokeColor);

	VectorGraphics::End();

	m_logs.clear();
	m_segments.clear();
	m_squares.clear();
}

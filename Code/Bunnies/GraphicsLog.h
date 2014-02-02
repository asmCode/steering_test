#ifndef GRAPHICS_LOG
#define GRAPHICS_LOG

#include <string>
#include <vector>
#include <Math/Vec3.h>
#include <Math/Matrix.h>

class SpriteBatch;
class FontRenderer;

class GraphicsLog
{
public:
	static void Initialize(SpriteBatch* spriteBatch, FontRenderer* fontRenderer);

	static void AddLog(const std::string& log);
	static void AddSegment(const sm::Vec3& begin, const sm::Vec3& end, const sm::Vec3& color = sm::Vec3(1, 1, 1));
	static void DrawSquare(const sm::Matrix& transform, const sm::Vec3& strokeColor = sm::Vec3(1, 1, 1));

	static void DrawAndClear();

private:
	class LogItem
	{
	public:
		std::string m_log;
	};

	class SegmentItem
	{
	public:
		sm::Vec3 m_begin;
		sm::Vec3 m_end;
		sm::Vec3 m_color;
	};

	class SquareItem
	{
	public:
		sm::Matrix m_transform;
		sm::Vec3 m_strokeColor;
	};

	static SpriteBatch *m_spriteBatch;
	static FontRenderer *m_font;

	static std::vector<LogItem> m_logs;
	static std::vector<SegmentItem> m_segments;
	static std::vector<SquareItem> m_squares;
};

#endif


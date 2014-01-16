#ifndef STREET_MAP
#define STREET_MAP

#include <Math/Matrix.h>
#include <string>
#include <Core/stdint.h>

#include "StreetPiece.h"

class StreetMap
{
public:
	enum PixelType
	{
		PixelType_Green,
		PixelType_Red,
		PixelType_Blue,
		PixelType_White,
	};

	static StreetMap *Instance;

	StreetMap(const std::string &dataPath);
	~StreetMap();

	uint32_t GetWidth() const;
	uint32_t GetHeight() const;

	StreetPiece::PieceType GetPieceType(uint32_t x, uint32_t y) const;

	bool GetRandomPavementArea(uint32_t x, uint32_t y, sm::Vec3 &position, sm::Vec3 &direction);

private:
	uint8_t *m_streetMap;

	uint32_t m_width;
	uint32_t m_height;

	PixelType GetPixelType(uint8_t *data, uint32_t x, uint32_t y);
};

#endif // STREET_MAP


#include "StreetMap.h"
#include "InterfaceProvider.h"
#include "Street.h"
#include "StreetPiece.h"
#include <Utils/Randomizer.h>

#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <Graphics/ImageLoader.h>
#include <assert.h>
#include <memory.h>
#include <stdlib.h>

StreetMap *StreetMap::Instance;

StreetMap::StreetMap(const std::string &dataPath)
{
	Instance = this;

	uint8_t *data;
	uint32_t bytesCount;

	if (!ImageLoader::LoadFromFile(dataPath + "textures/street_map2.png", data, m_width, m_height, bytesCount))
	{
		assert(false);
		return;
	}

	m_streetMap = new uint8_t[m_width * m_height];
	memset(m_streetMap, 0, m_width * m_height);

	for (uint32_t y = 1; y < m_height - 1; y++)
	{
		for (uint32_t x = 1; x < m_width - 1; x++)
		{
			PixelType pixelType = GetPixelType(data, x, y);
			PixelType pLeft = GetPixelType(data, x - 1, y);
			PixelType pRight = GetPixelType(data, x + 1, y);
			PixelType pUp = GetPixelType(data, x, y + 1);
			PixelType pDown = GetPixelType(data, x, y - 1);

			if (pixelType == PixelType_White)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_Pavement;
			else if ( // prosto poziomo
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight == PixelType_Green &&
				pUp!= PixelType_Green &&
				pDown != PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_StraightHori_1;
			else if ( // prosto pionowo
				pixelType == PixelType_Green &&
				pLeft != PixelType_Green &&
				pRight != PixelType_Green &&
				pUp == PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_StraightVert_1;
			else if ( // skret up prawo
				pixelType == PixelType_Green &&
				pLeft != PixelType_Green &&
				pRight == PixelType_Green &&
				pUp != PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TurnUpRight_1;
			else if ( // skret up lewo
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight != PixelType_Green &&
				pUp != PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TurnUpLeft_1;
			else if ( // skret down prawo
				pixelType == PixelType_Green &&
				pLeft != PixelType_Green &&
				pRight == PixelType_Green &&
				pUp == PixelType_Green &&
				pDown != PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TurnDownRight_1;
			else if ( // skret down lewo
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight != PixelType_Green &&
				pUp == PixelType_Green &&
				pDown != PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TurnDownLeft_1;
			else if ( // up T
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight == PixelType_Green &&
				pUp == PixelType_Green &&
				pDown != PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TUp_1;
			else if ( // down T
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight == PixelType_Green &&
				pUp != PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TDown_1;
			else if ( // left T
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight != PixelType_Green &&
				pUp == PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TLeft_1;
			else if ( // Right T
				pixelType == PixelType_Green &&
				pLeft != PixelType_Green &&
				pRight == PixelType_Green &&
				pUp == PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_TRight_1;
			else if ( // cross
				pixelType == PixelType_Green &&
				pLeft == PixelType_Green &&
				pRight == PixelType_Green &&
				pUp == PixelType_Green &&
				pDown == PixelType_Green)
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_Cross_1;

			else if ( // skycrapper
				pixelType == PixelType_Blue)
			{
				m_streetMap[y * m_width + x] = StreetPiece::PieceType_Skycrapper_1 + (rand() % 3);
			}
		}
	}

	delete [] data;
}

StreetMap::~StreetMap()
{
	delete [] m_streetMap;
}

uint32_t StreetMap::GetWidth() const
{
	return m_width;
}

uint32_t StreetMap::GetHeight() const
{
	return m_height;
}

StreetPiece::PieceType StreetMap::GetPieceType(uint32_t x, uint32_t y) const
{
	return (StreetPiece::PieceType)m_streetMap[y * m_width + x];
}

bool StreetMap::GetRandomPavementArea(uint32_t x, uint32_t y, sm::Vec3 &position, sm::Vec3 &direction)
{
	static Randomizer random;

	StreetPiece::PieceType type = GetPieceType(x, y);

	float streetWidth = 10.0f;
	float pavementWidth = 2.0f;

	switch (type)
	{
	case StreetPiece::PieceType_Pavement:
		return false;
		break;
	case StreetPiece::PieceType_StraightHori_1:
		position = sm::Vec3(
			x * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2),
			0,
			y * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * random.GetSign());

		direction = sm::Vec3(MathUtils::Sign(random.GetFloat(-1, 1)), 0, 0);
		return true;
		
	case StreetPiece::PieceType_StraightVert_1:
		position = sm::Vec3(
			x * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * random.GetSign(),
			0,
			y * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2));

		direction = sm::Vec3(0, 0, MathUtils::Sign(random.GetFloat(-1, 1)));
		return true;

	case StreetPiece::PieceType_TurnUpRight_1:
		if (random.GetInt(0, 1) == 1)
		{
			position = sm::Vec3(
				x * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2),
				0,
				y * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2));
		}
		else
		{
			position = sm::Vec3(
				x * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1,
				0,
				y * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2));
		}

		direction = sm::Vec3(0, 0, MathUtils::Sign(random.GetFloat(-1, 1)));
		return true;

	case StreetPiece::PieceType_TurnUpLeft_1:
		if (random.GetInt(0, 1) == 1)
		{
			position = sm::Vec3(
				random.GetFloat(-streetWidth / 2, streetWidth / 2),
				0,
				random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1);
		}
		else
		{
			position = sm::Vec3(
				random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1,
				0,
				random.GetFloat(-streetWidth / 2, streetWidth / 2));
		}

		position = Street::Instance->GetStreetPiece((uint8_t)type)->GetTransform() * position;
		position.x += streetWidth * x;
		position.z += streetWidth * y;

		direction = sm::Vec3(0, 0, MathUtils::Sign(random.GetFloat(-1, 1)));
		return true;

	case StreetPiece::PieceType_TurnDownRight_1:
		if (random.GetInt(0, 1) == 1)
		{
			position = sm::Vec3(
				random.GetFloat(-streetWidth / 2, streetWidth / 2),
				0,
				random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1);
		}
		else
		{
			position = sm::Vec3(
				random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1,
				0,
				random.GetFloat(-streetWidth / 2, streetWidth / 2));
		}

		position = Street::Instance->GetStreetPiece((uint8_t)type)->GetTransform() * position;
		position.x += streetWidth * x;
		position.z += streetWidth * y;

		direction = sm::Vec3(0, 0, MathUtils::Sign(random.GetFloat(-1, 1)));
		return true;

	case StreetPiece::PieceType_TurnDownLeft_1:
		if (random.GetInt(0, 1) == 1)
		{
			position = sm::Vec3(
				random.GetFloat(-streetWidth / 2, streetWidth / 2),
				0,
				random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1);
		}
		else
		{
			position = sm::Vec3(
				random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1,
				0,
				random.GetFloat(-streetWidth / 2, streetWidth / 2));
		}

		position = Street::Instance->GetStreetPiece((uint8_t)type)->GetTransform() * position;
		position.x += streetWidth * x;
		position.z += streetWidth * y;

		direction = sm::Vec3(0, 0, MathUtils::Sign(random.GetFloat(-1, 1)));
		return true;

	case StreetPiece::PieceType_Cross_1:
		break;

	case StreetPiece::PieceType_TUp_1:
		position = sm::Vec3(
			x * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2),
			0,
			y * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1);

		direction = sm::Vec3(MathUtils::Sign(random.GetFloat(-1, 1)), 0, 0);
		return true;

	case StreetPiece::PieceType_TDown_1:
		position = sm::Vec3(
			x * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2),
			0,
			y * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * 1);

		direction = sm::Vec3(MathUtils::Sign(random.GetFloat(-1, 1)), 0, 0);
		return true;

	case StreetPiece::PieceType_TLeft_1:
		position = sm::Vec3(
			x * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * 1,
			0,
			y * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2));

		direction = sm::Vec3(MathUtils::Sign(random.GetFloat(-1, 1)), 0, 0);
		return true;

	case StreetPiece::PieceType_TRight_1:
		position = sm::Vec3(
			x * streetWidth + random.GetFloat(streetWidth / 2 - pavementWidth, streetWidth / 2) * -1,
			0,
			y * streetWidth + random.GetFloat(-streetWidth / 2, streetWidth / 2));

		direction = sm::Vec3(MathUtils::Sign(random.GetFloat(-1, 1)), 0, 0);
		return true;

	case StreetPiece::PieceType_Skycrapper_1:
	case StreetPiece::PieceType_Skycrapper_2:
	case StreetPiece::PieceType_Skycrapper_3:
		break;
	default:
		break;
	}

	return false;
}

StreetMap::PixelType StreetMap::GetPixelType(uint8_t *data, uint32_t x, uint32_t y)
{
	uint8_t r = *(data + ((y * m_width + x) * 4) + 0);
	uint8_t g = *(data + ((y * m_width + x) * 4) + 1);
	uint8_t b = *(data + ((y * m_width + x) * 4) + 2);

	if (r > 240 && g == 0 && b == 0)
		return PixelType_Red;
	if (r == 0 && g > 240 && b == 0)
		return PixelType_Green;
	if (r == 0 && g == 0 && b > 240)
		return PixelType_Blue;
	if (r > 240 && g > 240 && b > 240)
		return PixelType_White;

	assert(false);
	return PixelType_White;
}


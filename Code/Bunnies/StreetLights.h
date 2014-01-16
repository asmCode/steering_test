#ifndef STREET_LIGHTS
#define STREET_LIGHTS

#include <Math/Vec3.h>
#include <Math/Matrix.h>
#include <Core/stdint.h>

#include "StreetPiece.h"

class StreetLights
{
public:
	enum LightColor
	{
		LightColor_Green,
		LightColor_Orange,
		LightColor_Red
	};

	StreetLights(const sm::Matrix& transform);

	LightColor GetLightColor() const;
	bool CanDrive() const;
	void SetLightColor(LightColor lightColor);
	const sm::Matrix& GetTransform() const;

	sm::Vec3 GetPosition() const;
	
private:
	sm::Matrix m_transform;

	LightColor m_lightColor;
};

#endif // STREET_LIGHTS


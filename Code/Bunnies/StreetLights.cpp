#include "StreetLights.h"

StreetLights::StreetLights(const sm::Matrix& transform) :
	m_transform(transform),
	m_lightColor(StreetLights::LightColor_Red)
{

}

StreetLights::LightColor StreetLights::GetLightColor() const
{
	return m_lightColor;
}

bool StreetLights::CanDrive() const
{
	return m_lightColor == StreetLights::LightColor_Green;
}

void StreetLights::SetLightColor(LightColor lightColor)
{
	m_lightColor = lightColor;
}

const sm::Matrix& StreetLights::GetTransform() const
{
	return m_transform;
}

sm::Vec3 StreetLights::GetPosition() const
{
	return m_transform * sm::Vec3(0, 0, 0);
}

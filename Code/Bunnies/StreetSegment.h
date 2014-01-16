#ifndef STREET_SEGMENT
#define STREET_SEGMENT

#include "StreetPath.h"
#include <Math/Matrix.h>
#include <Core/stdint.h>

class StreetPiece;
class StreetLights;

class StreetSegment
{
public:
	enum SegmentSide
	{
		SegmentSide_Left = 0,
		SegmentSide_Right,
		SegmentSide_Top,
		SegmentSide_Bottom
	};

	static const int MaxLights = 4;

	StreetSegment(const sm::Matrix &worldTransform, const sm::Vec3 &pivotPosition, StreetPiece *streetPiece);
	void Initialize();
	void SetVisibility(bool visibility);

	bool IsVisible() const;
	StreetPiece *GetStreetPiece();
	const sm::Vec3& GetPivotPosition() const;

	void Update(float seconds);

	uint32_t CoordX() const;
	uint32_t CoordY() const;

	StreetLights** GetStreetLights();

	const sm::Matrix& GetWorldTransform() const;

	StreetPath GetRandomPathAtPosition(const sm::Vec3& position);
	StreetLights* GetLights(const sm::Vec3& position);

private:
	static const float LightsInterval;

	int m_greenLightsIndex;
	float m_orangeCooldown;
	float m_lightsChangeCooldown;

	StreetPiece *m_streetPiece;
	sm::Vec3 m_pivotPosition;
	bool m_isVisible;

	sm::Matrix m_worldMatrix;

	std::vector<StreetPath> m_streetPaths;

	int m_lightsCount;
	StreetLights *m_streetLights[MaxLights];

	void InitializePaths();
	void InitializeLights();

	StreetLights* GetClosestLights(const sm::Vec3& position);

	SegmentSide GetSegmentSide(const sm::Vec3& position);
};

#endif // STREET_SEGMENT

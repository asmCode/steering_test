#ifndef STREET_SEGMENT
#define STREET_SEGMENT

#include <Math/Matrix.h>
#include <Core/stdint.h>

class StreetPiece;

class StreetSegment
{
public:
	StreetSegment(const sm::Matrix &worldTransform, const sm::Vec3 &pivotPosition, StreetPiece *streetPiece);

	void SetVisibility(bool visibility);

	bool IsVisible() const;
	StreetPiece *GetStreetPiece();
	const sm::Vec3& GetPivotPosition() const;

	uint32_t CoordX() const;
	uint32_t CoordY() const;

	const sm::Matrix& GetWorldTransform() const;

private:
	StreetPiece *m_streetPiece;
	sm::Vec3 m_pivotPosition;
	bool m_isVisible;

	sm::Matrix m_worldMatrix;
};

#endif // STREET_SEGMENT

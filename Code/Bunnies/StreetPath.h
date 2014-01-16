#pragma once

#include <Math/Vec3.h>
#include <vector>

class StreetSegment;
class StreetLights;

class StreetPath
{
public:
	StreetPath();

	void Initialize(StreetSegment *continousStreetSegment);
	void Rewind();
	sm::Vec3 GetNextPosition();
	sm::Vec3 GetBeginning();
	sm::Vec3 GetEnd();
	bool IsAtTheEnd() const;
	void AddPosition(const sm::Vec3& position);
	void SetStreetLights(StreetLights *streetLights);
	bool CanDrive() const;
	int GetIndex() const;
	StreetSegment* GetContinousSegment() const;
	sm::Vec3 GetPositionAtIndex(int index);

private:
	int m_index;

	std::vector<sm::Vec3> m_positions;

	StreetSegment *m_continousStreetSegment;

	StreetLights *m_streetLights;
};


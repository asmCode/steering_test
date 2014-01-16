#pragma once

#include <Math/Vec3.h>

class Car;
class Model;
class StreetSegment;
class BoxCollider;

class TrafficManager
{
public:
	TrafficManager();
	~TrafficManager();

	bool Initialize();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);

	void NotifyStreetSegmentVisibilityChanged(StreetSegment *streetSegment);

private:
	static const int MaxCars = 10;

	int m_activeCarsCount;

	Car* m_cars[MaxCars];

	Model *m_carModel;

	bool IsOnVisibleSegment(Car *car);

	void ActivateCar(Car *car, const sm::Vec3& position);
	void DeactivateCar(Car *car);
};


#ifndef BASE_CAR
#define BASE_CAR

#include <Math/Matrix.h>

class BaseCar
{
public:
	BaseCar(float axesWidth, float axesDistance, float maxSpeed);

	virtual ~BaseCar();

	void Update(float time, float seconds);

	void SetTurn(float turnValue);
	void SetAcceleration(float acc);

	const sm::Vec3& GetPosition() const;

	const sm::Matrix& GetTransform() const;

	const sm::Vec3& GetCarDirection() const;

	sm::Matrix m_worldMatrix;

	sm::Vec3 GetWheelsWorldDirection() const;

	void SetSpeed(float speed);

	void SetPosition(const sm::Vec3& position);

	void SetWheelsWorldDirection(const sm::Vec3& direction);
	void SetWheelsAngle(float wheelsAngle);

	float GetSpeed() const;
	float GetMaxSpeed() const;

	void SetCarDirection(const sm::Vec3& carDirection);

	const sm::Matrix& GetLeftWheelTransform() const;
	const sm::Matrix& GetRightWheelTransform() const;

	float GetAxesDistance() const;

protected:
	float m_maxSpeed;

	float m_turnValue;
	float m_acc;

	float m_axesWidth;
	float m_axesDistance;

	sm::Vec3 m_position;
	float m_speed;

	float m_wheelsAngle;
	sm::Vec3 m_carDirection;

	sm::Matrix m_leftWheelTransform;
	sm::Matrix m_rightWheelTransform;
};

#endif // BASE_CAR


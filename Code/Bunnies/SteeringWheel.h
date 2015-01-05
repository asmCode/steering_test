#pragma once

class SteeringWheel
{
public:
	SteeringWheel();
	
	void SetAngle();
	float GetAngle() const;



private:
	static float DefaultMaxAngle;
	static float DefaultSteerSpeed;
	static float DefaultSteerBackSpeed;

	float m_angle;
};


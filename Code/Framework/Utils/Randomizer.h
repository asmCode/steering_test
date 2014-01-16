#pragma once

class Randomizer
{
public:
	Randomizer();
	
	//min - max lacznie
	int GetInt(int min, int max);
	
	//0.0f - 1.0f lacznie
	float GetFloat();

	//-1.0f or 1.0f
	float GetSign();
	
	float GetFloat(float min, float max);
};

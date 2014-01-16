#include "Randomizer.h"
#include <stdlib.h>

Randomizer::Randomizer()
{
}
	
int Randomizer::GetInt(int min, int max)
{
	return ((int)rand() % (max - min + 1)) + min;
}

float Randomizer::GetFloat()
{
	return (float)rand() / (float)RAND_MAX;
}

float Randomizer::GetFloat(float min, float max)
{
	return (GetFloat() * (max - min)) + min;
}

float Randomizer::GetSign()
{
	return rand() % 2 == 0 ? 1.0f : -1.0f;
}

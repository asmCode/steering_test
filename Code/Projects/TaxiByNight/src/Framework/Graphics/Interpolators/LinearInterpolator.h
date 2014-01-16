#pragma once

#include "Interpolator.h"
#include "KeyFrame.h"
#include <vector>

template <typename Type>
class LinearInterpolator : public Interpolator<Type>
{
public:
	void GetValue(float time, Type &value)
	{
		int keysCount = static_cast<int>(this->keys.size());

		if (keysCount == 0)
			return;

		// if before or on first key
		if (time <= this->keys[0]->time)
		{
			value = this->keys[0]->value;
			return;
		}

		// if after or on last key
		if (time >= this->keys[keysCount - 1]->time)
		{
			value = this->keys[keysCount - 1]->value;
			return;
		}

		// if between first and last key
		int i = 0;
		for (i = 0; i < keysCount; i++)
		{
			if (this->keys[i]->time == time)
			{
				value = this->keys[i]->value;
				return;
			}

			if (this->keys[i]->time > time)
				break;
		}

		float normalizedTime = (time - this->keys[i - 1]->time) / (this->keys[i]->time - this->keys[i - 1]->time);
		value = (Type)((Type)this->keys[i - 1]->value * (1.0f - normalizedTime) + (Type)this->keys[i]->value * normalizedTime);
	}
};


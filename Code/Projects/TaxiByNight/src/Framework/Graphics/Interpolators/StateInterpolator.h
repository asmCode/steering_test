#pragma once

#include "Interpolator.h"
#include "KeyFrame.h"
#include <vector>

template <typename Type>
class StateInterpolator : public Interpolator<Type>
{
public:
	void GetValue(float time, Type &value)
	{
		if (this->keys.size() == 0)
		{
			//value = defaultValue;
			return;
		}

		if (time <= (*this->keys.begin()) ->time)
		{
			value = (*this->keys.begin()) ->value;
			return;
		}

		if (time >= (*(this->keys.end() - 1)) ->time)
		{
			value = (*(this->keys.end() - 1)) ->value;
			return;
		}

		//key on right side
		typename std::vector<KeyFrame<Type>*>::iterator rk;

		for (rk = this->keys.begin(); rk != this->keys.end(); rk++)
			if ((*rk) ->time > time)
				break;

		value = (*(rk - 1)) ->value;
	}
};

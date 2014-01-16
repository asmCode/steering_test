#pragma once

#include "IGameController.h"

class IGraphicsEngine;

class InfectedBunniesFactory
{
public:
	static IGameController* Create(IGraphicsEngine *graphicsEngine);
};


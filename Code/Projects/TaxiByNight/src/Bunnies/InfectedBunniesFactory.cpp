#include "InfectedBunniesFactory.h"
#include "GameController.h"

IGameController* InfectedBunniesFactory::Create(IGraphicsEngine *graphicsEngine)
{
	return new GameController(graphicsEngine);
}


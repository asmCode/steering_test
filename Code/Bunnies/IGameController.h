#ifndef _IGAME_CONTROLLER_
#define _IGAME_CONTROLLER_

#include <UI/IGestureHandler.h>
#include <Math/Vec3.h>
#include <string>

class Player;
class ISystemUtils;

class IGameController : public IGestureHandler
{
public:
	virtual ~IGameController() {}

	virtual bool Initialize(ISystemUtils *systemUtils) = 0;

	virtual void Draw(float time, float seconds) = 0;
	virtual void Update(float time, float seconds) = 0;

	virtual bool proto_IsInGame() = 0;
	virtual void proto_SetStrafeMove(float value) = 0;
	virtual void proto_SetForwardMove(float value) = 0;
	virtual void proto_SetLookTarget(const sm::Vec3 &lookTarget) = 0;

	virtual void HandleEnterForeground() = 0;
	virtual void HandleEnterBackground() = 0;
	virtual void HandleBackButton() = 0;
	virtual void HandleMenukButton() = 0;
};

#endif


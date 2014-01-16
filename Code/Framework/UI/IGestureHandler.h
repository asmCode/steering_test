#pragma once

#include <Math/Vec2.h>
#include <Core/stdint.h>

class IGestureHandler
{
public:
    enum GestureStatus
    {
        GestureStatus_Began,
        GestureStatus_Changed,
        GestureStatus_Ended
    };

    virtual ~IGestureHandler() {}

    virtual void HandlePinchGesture(IGestureHandler::GestureStatus status, float scale, float velocity) {}
    virtual void HandlePanGesture(IGestureHandler::GestureStatus status,
                                  const sm::Vec2 &pos,
                                  const sm::Vec2 &trans,
                                  const sm::Vec2 &velocity) {}
	//virtual void HandleTapGesture(const sm::Vec2 &point) {}
	virtual void HandleDoubleTapGesture(const sm::Vec2 &point) {}

	virtual void HandlePress(int pointId, const sm::Vec2 &point) {}
	virtual void HandleRelease(int pointId, const sm::Vec2 &point) {}
	virtual void HandleMove(int pointId, const sm::Vec2 &point) {}
};


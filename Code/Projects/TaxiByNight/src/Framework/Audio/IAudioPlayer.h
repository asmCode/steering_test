#pragma once

class IAudioPlayer
{
public:
	virtual ~IAudioPlayer() {}
	
	virtual void Play() = 0;
	virtual void Stop() = 0;
	
	virtual void SetLoop(bool loop) = 0;
	
	virtual void SetVolume(float vol) = 0;
	virtual float GetVolume() const = 0;
	virtual void SetPitch(float pitch) = 0;
};

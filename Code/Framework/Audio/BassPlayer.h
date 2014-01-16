#pragma once

#include "IAudioPlayer.h"
#include <bass/bass.h>

class BassPlayer : public IAudioPlayer
{
private:
	HSTREAM mp3Stream;

	static bool m_isInitialized;

public:
	BassPlayer(void);
	~BassPlayer(void);

	void LoadMusic(const char *filename);
	void Play();
	void Stop();
	void SetLoop(bool loop);
	void SetVolume(float vol);
	float GetVolume() const;
	void SetPitch(float pitch);

	void SetPosition(float time);
	float GetPosition();
};

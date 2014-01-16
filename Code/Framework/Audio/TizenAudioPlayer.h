#pragma once

#include "IAudioPlayer.h"
#include <FMedia.h>
#include <string>
#include <assert.h>

class TizenAudioPlayer :
	public IAudioPlayer,
	public Tizen::Media::IPlayerEventListener
{
public:
	TizenAudioPlayer();
	~TizenAudioPlayer();
	
	static TizenAudioPlayer* LoadFromFile(const std::string &file);
	
	void Play();
	void Stop();	
	void SetLoop(bool loop);
	void SetVolume(float vol);
	float GetVolume() const;
	void SetPitch(float pitch);

	virtual void OnPlayerOpened(result r) {}
	virtual void OnPlayerEndOfClip(void) {}
	virtual void OnPlayerBuffering(int percent) {}
	virtual void OnPlayerErrorOccurred(Tizen::Media::PlayerErrorReason r) {}
	virtual void OnPlayerInterrupted(void) {}
	virtual void OnPlayerReleased(void) {}
	virtual void OnPlayerSeekCompleted(result r) {}
	virtual void OnPlayerAudioFocusChanged (void) {}

private:
	 Tizen::Media::Player m_player;
};

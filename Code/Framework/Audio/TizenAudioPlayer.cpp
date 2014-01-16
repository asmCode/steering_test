#include "TizenAudioPlayer.h"
#include <Utils/Log.h>

TizenAudioPlayer::TizenAudioPlayer()
{
}

TizenAudioPlayer::~TizenAudioPlayer()
{
}

TizenAudioPlayer* TizenAudioPlayer::LoadFromFile(const std::string &file)
{
	TizenAudioPlayer *player = new TizenAudioPlayer();

	result r = E_SUCCESS;

	r = player->m_player.Construct(*player);
	if (IsFailed(r))
	{
		assert(false);
		return NULL;
	}

	r = player->m_player.OpenFile(file.c_str(), false);
	if (IsFailed(r))
	{
		assert(false);
		return NULL;
	}

	player->m_player.SetVolume(100);
										   	
	return player;
}

void TizenAudioPlayer::Play()
{
	m_player.Play();
}

void TizenAudioPlayer::Stop()
{
	m_player.Stop();
}

void TizenAudioPlayer::SetLoop(bool loop)
{
	m_player.SetLooping(loop);
}

void TizenAudioPlayer::SetVolume(float vol)
{
	m_player.SetVolume(vol);
}

float TizenAudioPlayer::GetVolume() const
{
	return (float)m_player.GetVolume() / 100.0f;
}

void TizenAudioPlayer::SetPitch(float pitch)
{
	// not implemented
}

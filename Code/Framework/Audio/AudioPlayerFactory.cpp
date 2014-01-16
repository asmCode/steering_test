#include "AudioPlayerFactory.h"

#ifndef _WIN32
#include "AlAudioPlayer.h"
#include "TizenAudioPlayer.h"
#else
#include "BassPlayer.h"
#endif

#include <assert.h>

IAudioPlayer* AudioPlayerFactory::CreateAlAudioPlayer(const std::string &file, bool stereo)
{
#ifndef _WIN32
	if (!AlAudioPlayer::IsInitialized())
	{
		if (!AlAudioPlayer::Initialize())
		{
			assert(false);
			return NULL;
		}
	}
	
	return AlAudioPlayer::LoadFromFile(file, stereo);
#else
	return CreateBassPlayer(file);
#endif
}

IAudioPlayer* AudioPlayerFactory::CreateTizenAudioPlayer(const std::string &file)
{
#ifndef _WIN32
	return TizenAudioPlayer::LoadFromFile(file);
#else
	return CreateBassPlayer(file);
#endif
}

#ifdef _WIN32
IAudioPlayer* AudioPlayerFactory::CreateBassPlayer(const std::string &file)
{
	BassPlayer *player = new BassPlayer();
	player->LoadMusic(file.c_str());

	return player;
}
#endif

#pragma once

#include <string>
#include "IAudioPlayer.h"

class AudioPlayerFactory
{
public:
	static IAudioPlayer* CreateAlAudioPlayer(const std::string &file, bool stereo);
	static IAudioPlayer* CreateTizenAudioPlayer(const std::string &file);
	static IAudioPlayer* CreateBassPlayer(const std::string &file);
};

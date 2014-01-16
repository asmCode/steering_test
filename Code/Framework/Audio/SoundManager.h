#pragma once

#include <string>
#include "IAudioPlayer.h"

#undef PlaySoundA
#undef PlaySound

class SoundManager
{
public:
	enum Sound
	{
		Sound_Collision01 = 0,
		Sound_Collision02,
		Sound_Collision03,
		Sound_Button,
		Sound_Money,
		Sound_Summary,
		Sound_Doors,
		Sound_Die,
		
		SoundsCount // always last
	};
	
	static const std::string SoundFiles[];
	
private:	
	static SoundManager *m_instance;

	float soundVolume;
	float musicVolume;
	
	IAudioPlayer *music;
	IAudioPlayer *m_engine;
	IAudioPlayer *sounds[SoundsCount];
		
	SoundManager();

public:
	~SoundManager();

	static SoundManager *GetInstance();

	bool Initialize(const std::string &audioPath);
	
	void SetSoundVolume(float value);
	void SetMusicVolume(float value);

	float GetSoundVolume() const;
	float GetMusicVolume() const;
	
	void PlayMusic();
	void StopMusic();
	
	void StartEngine();
	void StopEngine();
	void SetEnginePitch(float pitch);
	
	void PlaySound(Sound sound);
	
	bool IsLoaded() const;
};

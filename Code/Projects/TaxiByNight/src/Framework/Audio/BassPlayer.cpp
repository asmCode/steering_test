#ifdef _WIN32

#include "BassPlayer.h"
#include <stddef.h>
#include <assert.h>

bool BassPlayer::m_isInitialized;

BassPlayer::BassPlayer(void)
{
	mp3Stream = NULL;
}

BassPlayer::~BassPlayer(void)
{
}

void BassPlayer::LoadMusic(const char *filename)
{
	if (!m_isInitialized)
	{
		BOOL results;
		results = BASS_Init(-1, 44100, 0, NULL, NULL);
		if (results)
		{
			m_isInitialized = true;
		}
		else
		{
			int error = BASS_ErrorGetCode();
			assert(false);
		}
	}

	mp3Stream = BASS_StreamCreateFile(false, filename, 0, 0, 0);
}

void BassPlayer::Play()
{
	BASS_ChannelPlay(mp3Stream, false);
}

void BassPlayer::Stop()
{
	BASS_ChannelStop(mp3Stream);
}

void BassPlayer::SetPosition(float time)
{
	QWORD bytesPos = BASS_ChannelSeconds2Bytes(mp3Stream, time);
	BOOL result = BASS_ChannelSetPosition(mp3Stream, bytesPos, BASS_POS_BYTE);

	if (result != TRUE)
	{
		int errorCode = BASS_ErrorGetCode();
		assert(false);
	}
}

float BassPlayer::GetPosition()
{
	QWORD bytesPos = BASS_ChannelGetPosition(mp3Stream, BASS_POS_BYTE);
	return (float)BASS_ChannelBytes2Seconds(mp3Stream, bytesPos);
}

void BassPlayer::SetLoop(bool loop)
{
	if (loop)
		BASS_ChannelFlags(mp3Stream, BASS_SAMPLE_LOOP, BASS_SAMPLE_LOOP);
	else
		BASS_ChannelFlags(mp3Stream, 0, BASS_SAMPLE_LOOP);
}

void BassPlayer::SetVolume(float vol)
{
	BASS_ChannelSetAttribute(mp3Stream, BASS_ATTRIB_VOL, vol);
}

float BassPlayer::GetVolume() const
{
	return 1.0f;
}

void BassPlayer::SetPitch(float pitch)
{
	BASS_ChannelSetAttribute(mp3Stream, BASS_ATTRIB_FREQ, 44100.0f * pitch);
}

#endif

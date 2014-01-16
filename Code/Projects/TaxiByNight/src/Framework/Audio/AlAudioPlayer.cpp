#ifndef _WIN32
#include "AlAudioPlayer.h"
#include <math.h>
#include <FIo.h>
#include <Utils/Log.h>
#include <assert.h>

using namespace Tizen::Base;
using namespace Tizen::Base::Runtime;
using namespace Tizen::Locales;
using namespace Tizen::System;
using namespace Tizen::App;
using namespace Tizen::System;
using namespace Tizen::Io;
using namespace Tizen::Media;

bool AlAudioPlayer::m_isInitialized;
ALCdevice *AlAudioPlayer::m_audioDev;
ALCcontext *AlAudioPlayer::m_audioContext;

bool AlAudioPlayer::IsInitialized()
{
	return m_isInitialized;
}

bool AlAudioPlayer::Initialize()
{
	ALenum openAlResult = AL_NO_ERROR;

	alutInit(0,0);
	openAlResult = alGetError();
	if (openAlResult != AL_NO_ERROR)
	{
		AppLog("alutInit Failed with %d", openAlResult);
		return false;
	}

	m_isInitialized = true;

	return true;
}

AlAudioPlayer* AlAudioPlayer::LoadFromFile(const std::string &file, bool stereo)
{
	AlAudioPlayer *player = new AlAudioPlayer();
	player->Construct(file.c_str());

	return player;
}

AlAudioPlayer::AlAudioPlayer()
{
	__alSource = 0;
	__openalPlayerState = OPENAL_PLAYER_STATE_NONE;
}
AlAudioPlayer::~AlAudioPlayer()
{
	if (__alSource)
	{
		alDeleteSources(1, &__alSource);
	}

	if (m_bufferName != 0)
		alDeleteBuffers(1, &m_bufferName);
}
result
AlAudioPlayer::ConvertError(ALenum error)
{
	switch(error)
	{
	case AL_NO_ERROR:
		return E_SUCCESS;
		break;
	case AL_INVALID_NAME:
		return E_INVALID_ARG;
		break;
	case AL_INVALID_ENUM:
		return E_OUT_OF_RANGE;
		break;
	case AL_INVALID_VALUE:
		return E_INVALID_ARG;
		break;
	case AL_INVALID_OPERATION:
		return E_INVALID_OPERATION;;
		break;
	case AL_OUT_OF_MEMORY:
		return E_OUT_OF_MEMORY;
	default:
		return E_FAILURE;
	}
}
/**
 * Construct OpenAL based player.
 * @return An error code
 * @param[in] filePath  The source file path that will be played.
 * @remarks If the filePath is wav file, then whole data will be loaded into memory.
 * @remarks For the compressed data, if the file size is less than some specific threshold the it will be decoded into memory.
 */
result
AlAudioPlayer::Construct(const Tizen::Base::String &filePath)
{
	result r = E_SUCCESS;
	Tizen::Io::FileAttributes attr;
	ALenum openAlResult = AL_NO_ERROR;

	alGenSources(1, &__alSource);
	openAlResult = alGetError();
	if (openAlResult != AL_NO_ERROR)
	{
		AppLog("alGenSources Failed with %d", openAlResult);
		return ConvertError(openAlResult);
	}
	__channelType = AUDIO_CHANNEL_TYPE_NONE;

	r = Tizen::Io::File::GetAttributes(filePath, attr);
	TryReturn(r == E_SUCCESS, r, "file.GetAttributes:%s %ls", GetErrorMessage(r), filePath.GetPointer());

	String ext =  Tizen::Io::File::GetFileExtension(filePath);

	// compressed data
	Tizen::Io::File file;
	__codecType = CODEC_NONE;

	r = file.Construct(filePath, "r");
	TryReturn(r == E_SUCCESS, r, "file.Construct:%s %ls", GetErrorMessage(r), filePath.GetPointer());

	ByteBuffer __srcBuf;
	r = __srcBuf.Construct((int)attr.GetFileSize());
	TryReturn(r == E_SUCCESS, r, "buffer.Construct:%s %d", GetErrorMessage(r), (int)attr.GetFileSize());
	file.Read(__srcBuf);
	__srcBuf.Flip();

	if (ext.Equals(L"mp3", false))
	{
		__codecType = CODEC_MP3;
	}
	else if (ext.Equals(L"aac", false))
	{
		__codecType = CODEC_AAC;
	}
	else
	{
		AppLog("Unknown format %ls", filePath.GetPointer());
		return E_UNSUPPORTED_FORMAT;
	}

	alGenBuffers(1, &m_bufferName);
	FillAlBuffer(__srcBuf, m_bufferName);

	alSourcef (__alSource, AL_PITCH, 1.0);
	alSourcef (__alSource, AL_GAIN, 1.0);
	alSource3f(__alSource, AL_POSITION, 0.0, 0.0, 0.0);
	alSource3f(__alSource, AL_VELOCITY, 0.0, 0.0, 0.0);
	alSource3f(__alSource, AL_DIRECTION, 0.0, 0.0, 0.0);
	alSourcef(__alSource, AL_ROLLOFF_FACTOR, 0.0);
	alSourcei(__alSource, AL_SOURCE_RELATIVE, AL_TRUE);
	openAlResult = alGetError();
	if (openAlResult != AL_NO_ERROR)
	{
		AppLog("alSourcei Failed with %d", openAlResult);
		return ConvertError(openAlResult);
	}
	__openalPlayerState = OPENAL_PLAYER_STATE_INITIALIZED;
	return r;
}

void AlAudioPlayer::Play()
{
	alSourcePlay(__alSource);
	ALenum openAlResult = alGetError();
	if (openAlResult != AL_NO_ERROR)
	{
		AppLog("alSourcePlay Failed with %d", openAlResult);
		assert(false);
	}

	__openalPlayerState = OPENAL_PLAYER_STATE_PLAYING;
}

void
AlAudioPlayer::Stop()
{
	alSourceStop(__alSource);

	__openalPlayerState = OPENAL_PLAYER_STATE_STOPPED;
}

void
AlAudioPlayer::CleanUp(void)
{
	if (__openalPlayerState != OPENAL_PLAYER_STATE_NONE)
	{
		alDeleteSources(1, &__alSource);
		alutExit();
		__openalPlayerState = OPENAL_PLAYER_STATE_NONE;
	}
}

void AlAudioPlayer::FillAlBuffer(ByteBuffer &srcBuf, ALuint alBuffer)
{
	AudioDecoder audioDecoder;

	result r = E_SUCCESS;

	r = audioDecoder.Construct(__codecType, null);
	assert(r == E_SUCCESS);
	r = audioDecoder.Probe(srcBuf, __sampleType, __channelType, __sampleRate);
	assert(r == E_SUCCESS);

	assert(__channelType != AUDIO_CHANNEL_TYPE_NONE);
	assert(__sampleRate != 0);

	if(__channelType == AUDIO_CHANNEL_TYPE_MONO)
	{
		__alFormat = AL_FORMAT_MONO16;
	}
	else
	{
		__alFormat = AL_FORMAT_STEREO16;
	}

	ByteBuffer __pcmBuf;
	__pcmBuf.Construct(MAX_AL_BUFFER_SIZE * 100);

	while ( srcBuf.GetRemaining() > 0)
	{
		r = audioDecoder.Decode(srcBuf, __pcmBuf);
		if (IsFailed(r))
		{
			AppLog(GetErrorMessage(r));
			assert(false);
			break;
		}
	}

	alBufferData(alBuffer, __alFormat, __pcmBuf.GetPointer(), __pcmBuf.GetPosition(), __sampleRate);
	AppLog("Filling AL buffer with size %d", __pcmBuf.GetPosition());
	ALenum openAlResult = alGetError();
	if (openAlResult != AL_NO_ERROR)
	{
		assert(false);
	}

	alSourcei(__alSource, AL_BUFFER, alBuffer);
	openAlResult = alGetError();
	if (openAlResult != AL_NO_ERROR)
	{
		assert(false);
	}
}

void AlAudioPlayer::SetLoop(bool loop)
{
	alSourcei(__alSource, AL_LOOPING, loop ? AL_TRUE : AL_FALSE);
}

void AlAudioPlayer::SetPitch(float pitch)
{
	alSourcef (__alSource, AL_PITCH, pitch);
}

void AlAudioPlayer::SetVolume(float vol)
{
	alSourcef(__alSource, AL_GAIN, (ALfloat)vol);
}

float AlAudioPlayer::GetVolume() const
{
	float vol;
	alGetSourcef(__alSource, AL_GAIN, &vol);
	return vol;
}

#endif

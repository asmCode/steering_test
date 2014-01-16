#ifndef _WIN32

#include "IAudioPlayer.h"
#include <new>
#include <unique_ptr.h>
#include <FBase.h>
#include <FSystem.h>
#include <FApp.h>
#include <FMedia.h>
#include <AL/alut.h>
#include <AL/al.h>
#include <string>

#define MAX_AL_BUFFER_SIZE  (1024*16)


class AlAudioPlayer : public IAudioPlayer
{
public:
	enum OpenAlPlayerState{
		OPENAL_PLAYER_STATE_NONE,
		OPENAL_PLAYER_STATE_INITIALIZED,
		OPENAL_PLAYER_STATE_PLAYING,
		OPENAL_PLAYER_STATE_PAUSED,
		OPENAL_PLAYER_STATE_STOPPED
	};
	AlAudioPlayer() ;
	virtual ~AlAudioPlayer();
	result Construct(const Tizen::Base::String &filePath);
	void Play(void);
	void Stop(void);
	void CleanUp(void);
	bool Update(void);
	OpenAlPlayerState GetState(void){return __openalPlayerState;}

	void SetLoop(bool loop);
	void SetVolume(float vol);
	float GetVolume() const;
	void SetPitch(float pitch);

	static bool IsInitialized();
	static bool Initialize();
	static AlAudioPlayer* LoadFromFile(const std::string &file, bool stereo);

protected:
	static const int MAX_FILE_NAME = 256;
	result ConvertError(ALenum error);
	void FillAlBuffer(Tizen::Base::ByteBuffer &srcBuf, ALuint alBuffer);

private:
	static bool m_isInitialized;

	static ALCdevice *m_audioDev;
	static ALCcontext *m_audioContext;

	ALuint m_bufferName;
	ALuint __alSource;
	ALenum __alFormat;

	Tizen::Media::CodecType __codecType;
	Tizen::Media::AudioChannelType __channelType;
	Tizen::Media::AudioSampleType __sampleType;
	int __sampleRate;
	OpenAlPlayerState __openalPlayerState;
};

#endif
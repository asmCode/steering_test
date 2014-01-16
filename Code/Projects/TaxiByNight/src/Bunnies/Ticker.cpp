#include "Ticker.h"

Ticker::Ticker(void) :
	m_timeLeft(0.0f),
	m_baseTime(0.0f)
{
}

Ticker::Ticker(float time) :
	m_timeLeft(time),
	m_baseTime(time)
{
}

Ticker::~Ticker(void)
{
}

void Ticker::SetTicker(float time)
{
	m_timeLeft = time;
	m_baseTime = time;
}

void Ticker::Progress(float seconds)
{
	m_timeLeft -= seconds;
	if (m_timeLeft < 0.0f)
		m_timeLeft = 0.0f;
}

bool Ticker::IsTimeout() const
{
	return m_timeLeft == 0.0f;
}

void Ticker::Reset()
{
	m_timeLeft = m_baseTime;
}


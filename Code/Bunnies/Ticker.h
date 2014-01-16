#pragma once

class Ticker
{
public:
	Ticker(void);
	explicit Ticker(float time);

	~Ticker(void);

	void SetTicker(float time);
	void Progress(float seconds);
	bool IsTimeout() const;
	void Reset();

private:
	float m_baseTime;
	float m_timeLeft;
};


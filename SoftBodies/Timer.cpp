#include "Timer.h"
#include <SDL_timer.h>
#include <algorithm>

Timer::Timer() : frameStart(0), lastFrame(0), frameTime(0)
{
}

unsigned int Timer::computeDeltaTime()
{
	frameStart = SDL_GetTicks();
	unsigned int dt = frameStart - lastFrame;
	lastFrame = frameStart;
	return std::min(dt, MAX_DT);    // Clamp delta time
}

void Timer::delayTime()
{
	frameTime = SDL_GetTicks() - frameStart;
	if (frameTime < FRAME_DELAY) {
		SDL_Delay(FRAME_DELAY - frameTime);
	}
}

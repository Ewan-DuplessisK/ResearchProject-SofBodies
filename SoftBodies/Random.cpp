#include "Random.h"

void Random::init()
{
	std::random_device rd;
	Random::seed(rd());
}

void Random::seed(unsigned int seed)
{
	sGenerator.seed(seed);
}

float Random::getFloat()
{
	return getFloatRange(0.0f, 1.0f);
}

float Random::getFloatRange(float min, float max)
{
	std::uniform_real_distribution<float> dist(min, max);
	return dist(sGenerator);
}

int Random::getIntRange(int min, int max)
{
	std::uniform_int_distribution<int> dist(min, max);
	return dist(sGenerator);
}

Vector2 Random::getVector(const Vector2& min, const Vector2& max)
{
	Vector2 r = Vector2(getFloat(), getFloat());
	Vector2 diff = max - min;
	return min + Vector2(diff.x * r.x, diff.y * r.y);
}

std::mt19937 Random::sGenerator;

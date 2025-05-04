#include "Game.h"
int main(int argc, char** argv)
{
	bool isGameInit = Game::instance().initialize();
	if (isGameInit)
	{
		Game::instance().load();
		Game::instance().loop();
		Game::instance().unload();
	}
	Game::instance().close();
	return 0;
}
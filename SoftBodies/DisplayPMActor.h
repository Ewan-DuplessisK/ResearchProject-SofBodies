#pragma once
#include "Actor.h"

class DisplayPMActor : public Actor{
public:
    DisplayPMActor();
private:
    class MeshComponent* mesh;
};

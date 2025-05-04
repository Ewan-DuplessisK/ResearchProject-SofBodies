#include "CubeActor.h"
#include "MeshComponent.h"
#include "Assets.h"

CubeActor::CubeActor()
{
	MeshComponent* mc = new MeshComponent(this);
	mc->setMesh(Assets::getMesh("Mesh_Cube"));
}

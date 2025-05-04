#include "SphereActor.h"
#include "MeshComponent.h"
#include "Assets.h"

SphereActor::SphereActor() : Actor()
{
	MeshComponent* mc = new MeshComponent(this);
	mc->setMesh(Assets::getMesh("Mesh_Sphere"));
}

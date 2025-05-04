#include "DisplayPMActor.h"

#include "Assets.h"
#include "MeshComponent.h"

DisplayPMActor::DisplayPMActor(){
    mesh = new MeshComponent(this);
    mesh->setMesh(Assets::getMesh("Mesh_Sphere"));
    setScale(.2f);
}

#pragma once
#include "Actor.h"

struct Point;
struct Neighbor;

struct Point{
    Vector3 position;
    Vector3 force;
    Vector3 speed;
    float mass;
    std::vector<Neighbor> neighbors;
};

struct Neighbor{
    Point* point;
    float baseDistance;
    float limits[2];
    float maxSpringForce;
};

class SoftBody : public Actor{
public:
    SoftBody();
    void updateActor(float dt) override;
    void setUniformMovement(Vector3 pUniform){uniformMovement=pUniform;}
    std::vector<Point> points;

private:
    
    Vector3 uniformMovement;

    void applyUniformMovement(float dt);
    void SimpleSphereResolve(float dt);
};

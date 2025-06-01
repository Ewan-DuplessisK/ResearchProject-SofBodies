#pragma once
#include "Actor.h"

struct Point;
struct Spring;

struct Point{
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
    const float radius = 5.f;
};
//each frame: force=0 -> force+=gravityAcc*mass +=springs +=external ; speed+=(force*deltatime)/mass ; position+=speed*deltatime (euler integration)

struct Spring{
    Point* points[2];
    float baseDistance;
    float stiffness;
    float damping;
};

class SoftBody : public Actor{
public:
    void Draw();
    SoftBody();
    void updateActor(float dt) override;
    void setUniformMovement(Vector3 pUniform){uniformMovement=pUniform;}
    bool pointMassGroundCollisions(Point* p);
    std::vector<Point> points;
    std::vector<Vector3> restPositions;

private:
    
    Vector3 uniformMovement;
    std::vector<Spring> springs;

    void SolveSpring(Spring spring);
    void ClampSpringForce(Spring& spring);
    void ApplyShapeMatching(float stiffness);
    void Solve(float dt);
};

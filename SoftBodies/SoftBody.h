#pragma once
#include "Actor.h"

struct Point;
//struct Neighbor;
struct Spring;

struct Point{
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
    //std::vector<Neighbor> neighbors;
};
//each frame: force=0 -> force+=gravityAcc*mass +=springs +=external ; speed+=(force*deltatime)/mass ; position+=speed*deltatime (euler integration)

struct Spring{
    Point* points[2];
    float baseDistance;
    float stiffness;
    float damping;
};

/*struct Neighbor{
    Point* point;
    float baseDistance;
    float limits[2];
    float maxSpringForce;
};*/

class SoftBody : public Actor{
public:
    SoftBody();
    void updateActor(float dt) override;
    void setUniformMovement(Vector3 pUniform){uniformMovement=pUniform;}
    bool pointMassGroundCollisions(Point* p);
    std::vector<Point> points;

private:
    
    Vector3 uniformMovement;
    std::vector<Spring> springs;

    //void applyUniformMovement(float dt);
    void SolveSpring(Spring spring);
    void Solve(float dt);
};

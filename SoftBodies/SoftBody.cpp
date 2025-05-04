#include "SoftBody.h"

#include "AABB.h"
#include "BoxComponent.h"
#include "Collisions.h"
#include "LineSegment.h"
#include "PhysicsSystem.h"
#include "Game.h"
#include "MeshComponent.h"

void SoftBody::updateActor(float dt){
    Actor::updateActor(dt);
    
    Solve(dt);

    //std::cout<<Vector3(points[1].position-points[0].position).length()<<std::endl;
}

bool SoftBody::pointMassGroundCollisions(Point* p){
    auto& groundPlane = getGame().getPlanes()[0];
    const AABB& planeBox = groundPlane->getBox()->getWorldBox();
    if(Collisions::intersect(Sphere{p->position,5},planeBox)){
        //std::cout<<"planecollisions"<<std::endl;
        p->force.z = /*planeBox.max.z - (p->position.z - 5.f)*/9*p->mass;
        p->position.z = planeBox.max.z + 5.f;
        //std::cout<<p->position.z<<std::endl;
        return true;
    }
    return false;
}

/*void SoftBody::applyUniformMovement(float dt){
    for(auto p:points){
        Vector3 tSpeed = p.speed + uniformMovement;
        Vector3 tPosition;
        Vector3 start = p.position;
        Vector3 end = start + tSpeed*dt;

        LineSegment line(start,end);
        PhysicsSystem::CollisionInfo info;
        if(getGame().getPhysicsSystem().segmentCast(line, info)){
            tPosition = info.point;
            Vector3 normalNullification = info.normal * Vector3::dot(tSpeed,info.normal);
            float ratio = (tPosition - p.position).length()/normalNullification.length();
            tSpeed -= normalNullification;
            tPosition += tSpeed;

            p.position = tPosition;
            p.speed -= info.normal * ratio;
        }
    }
}*/

/*void SoftBody::SimpleSphereResolve(float dt){
    Point center = points[0];
    center.speed = Vector3();
    for(Neighbor neighbor : center.neighbors){// center is neighbor to all outer points
        Vector3 relativePosition = neighbor.point->position-center.position;
        float distance = relativePosition.length();
        float ratio = distance/neighbor.baseDistance;
        float force = -2*ratio+2; //<1 compressed:positive force, >1 extended:negative force, max out at 50%
        relativePosition.normalize();
        center.speed += relativePosition*force;
    }
    center.speed *= 1.f/center.neighbors.size();
    points[0].speed = center.speed;
}*/

//wont work for simple sphere
/*void SoftBody::applySpringConstraints(float dt){
    for(auto p : points){
        for(auto n : p.neighbors){
            Vector3 ab = p.position-n.point->position;
            if(ab.length()<n.limits[0]){ //if points too close
                float diff = n.limits[0]-ab.length();
                ab.normalize();
                Vector3 ba = -ab;
                float ratio = p.speed.length()/(n.point->speed.length()*Vector3::dot(p.speed,n.point->speed)); //I think
                p.position += ba*diff*ratio;
                n.point->position += ab*diff*(1-ratio);
            }
        }
    }
}*/

void SoftBody::SolveSpring(Spring spring){
    Vector3 ab = Vector3(spring.points[1]->position-spring.points[0]->position);
    Vector3 abNorm = ab;
    abNorm.normalize();
    
    float springForce = (ab.length() - spring.baseDistance) * spring.stiffness;

    Vector3 velDiff = spring.points[1]->speed-spring.points[0]->speed;

    float dot = Vector3::dot(abNorm,velDiff);

    float dampingForce = dot*spring.damping;

    float totalForce = springForce + dampingForce;

    Vector3 aForce = totalForce*abNorm;
    Vector3 baNorm = -1.f*ab;
    baNorm.normalize();
    Vector3 bForce = totalForce * baNorm;
    // >0 attraction <0 repulsion

    spring.points[0]->force+=aForce;
    spring.points[1]->force+=bForce;
}

void SoftBody::Solve(float dt){
    for(int i=0;i<points.size();i++){
        if(!pointMassGroundCollisions(&points[i]))points[i].force=Vector3::unitZ*-9*points[i].mass; //This line for gravity
        //points[i].force=Vector3(); //This line for no gravity 
    }
    for(int i=0;i<springs.size();i++){
        SolveSpring(springs[i]);
    }
    for(int i=0;i<points.size();i++){
        points[i].speed+=points[i].force*dt * (1/points[i].mass);
        points[i].position+=points[i].speed*dt;
    }
    //float tmp = (points[0].position-points[2].position).length();
    //std::cout<<tmp<<std::endl;
}

SoftBody::SoftBody(){
    //Hexagon
    points.emplace_back(Point{Vector3{0,.5f*20.f,3.5f/4.f*20.f+30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{0,1.f*20.f,30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{0,.5f*20.f,-3.5f/4.f*20.f+30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{0,-.5f*20.f,-3.5f/4.f*20.f+30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{0,-1.f*20.f,30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{0,-.5f*20.f,3.5f/4.f*20.f+30.f},Vector3(),Vector3(),2});

    //Base cycle
    Spring s;
    s.points[0]=&points[0];
    s.points[1]=&points[1];
    s.baseDistance=10.f;
    s.stiffness=10.f;
    s.damping=1.f;
    springs.emplace_back(s);
    Spring s2;
    s2.points[0]=&points[1];
    s2.points[1]=&points[2];
    s2.baseDistance=10.f;
    s2.stiffness=10.f;
    s2.damping=1.f;
    springs.emplace_back(s2);
    Spring s3;
    s3.points[0]=&points[2];
    s3.points[1]=&points[3];
    s3.baseDistance=10.f;
    s3.stiffness=10.f;
    s3.damping=1.f;
    springs.emplace_back(s3);
    Spring s4;
    s4.points[0]=&points[3];
    s4.points[1]=&points[4];
    s4.baseDistance=10.f;
    s4.stiffness=10.f;
    s4.damping=1.f;
    springs.emplace_back(s4);
    Spring s5;
    s5.points[0]=&points[4];
    s5.points[1]=&points[5];
    s5.baseDistance=10.f;
    s5.stiffness=10.f;
    s5.damping=1.f;
    springs.emplace_back(s5);
    Spring s6;
    s6.points[0]=&points[5];
    s6.points[1]=&points[0];
    s6.baseDistance=10.f;
    s6.stiffness=10.f;
    s6.damping=1.f;
    springs.emplace_back(s6);


    //Pyramid
    /*points.emplace_back(Point{Vector3{0,0,50.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{28.f,0.f,30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{-10.f,10.f,30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{-10.f,-10.f,30.f},Vector3(),Vector3(),2});

    Spring s0;
    s0.points[0]=&points[0];
    s0.points[1]=&points[1];
    s0.baseDistance=25.f;
    s0.stiffness=10.f;
    s0.damping=2.f;
    springs.emplace_back(s0);
    Spring s1;
    s1.points[0]=&points[0];
    s1.points[1]=&points[2];
    s1.baseDistance=25.f;
    s1.stiffness=10.f;
    s1.damping=2.f;
    springs.emplace_back(s1);
    Spring s2;
    s2.points[0]=&points[0];
    s2.points[1]=&points[2];
    s2.baseDistance=25.f;
    s2.stiffness=10.f;
    s2.damping=2.f;
    springs.emplace_back(s2);
    Spring s3;
    s3.points[0]=&points[0];
    s3.points[1]=&points[3];
    s3.baseDistance=25.f;
    s3.stiffness=10.f;
    s3.damping=2.f;
    springs.emplace_back(s3);

    Spring s4;
    s4.points[0]=&points[1];
    s4.points[1]=&points[2];
    s4.baseDistance=25.f;
    s4.stiffness=10.f;
    s4.damping=2.f;
    springs.emplace_back(s4);
    Spring s5;
    s5.points[0]=&points[2];
    s5.points[1]=&points[3];
    s5.baseDistance=25.f;
    s5.stiffness=10.f;
    s5.damping=2.f;
    springs.emplace_back(s5);
    Spring s6;
    s6.points[0]=&points[3];
    s6.points[1]=&points[1];
    s6.baseDistance=25.f;
    s6.stiffness=10.f;
    s6.damping=2.f;
    springs.emplace_back(s6);*/
}


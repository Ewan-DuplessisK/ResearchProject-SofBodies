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
}

void SoftBody::applyUniformMovement(float dt){
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
}

void SoftBody::SimpleSphereResolve(float dt){
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
}

//wont work for simple sphere
void SoftBody::applySpringConstraints(float dt){
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
}



SoftBody::SoftBody(){
    
}


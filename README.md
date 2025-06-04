# Realtime Soft Body Dynamics, a Research Project

## Intro

Realtime Soft Body Dynamics is becoming better year after year, being more precise, more efficient, with recent advancement to computation methods like the Materail Point Method, or Deep-Learning. But while those methods can be made light enough to run in real-time, they still require quite a lot of operations. They're great for high fidelity simlation in a physics engine, but they're often too heavy to run as part of a game engine, alongside plenty of other systems that require computational power. As such for soft bodies in games the focus is on low-cost, low- to mid-fidelity methods. They don't have to be perfect, they just need to be good enough, to shox to the players what the games creators wanted, at the lowest possible cost in terms of performances. As such some simpler methods tend to be preferred, for example Extended Position Based Dynamics (XPBD) and its more recent upgrades like Position-Based Dynamics with Smoothing Kernels Handles Continuum Inelasticity (XPBI).

Going from methods to tools, there are physics engines and solvers that can be integrated into game engines, such as Havok, Bullet, or Jelly Engine. Some commercial engines also have their own pre-integrated Soft Body physics, such as Unreal Engine with Chaos Flesh or Godot with their SoftBody3d nodes.

## Approach

Simulation methods lixe XPBD are quite heavy when it comes to maths, and it's not my strong point. So to gain an initial understanding of how Soft Bodies Dynamics work, I started by trying to implement a simpler simulation.

I first tried to make a sphere out of springs, all connected to a center point. There was no springs structure, everything was calculated in place. But there were plenty of issues with my maths and halway through I realised that this method wouldn't work, so I made a proper string structure and changed my approach by calculating the spring constraints separatedly. But this still wasn't working, understanding that I wouldn't go much further using only my primitive knowledge of physics simulations, I did some more research and started working on a version using Euler Integration, which is the code that is currently working.

### Euler Integration

In this version Point and String are two proper structures that look like this:
```
struct Point{
    Vector3 position;
    Vector3 speed;
    Vector3 force;
    float mass;
};
struct Spring{
    Point* points[2];
    float baseDistance;
    float stiffness;
    float damping;
};
```
The first step of the Solver in a frame is to reset all the forces to 0, on each points, and to be a bit more efficient gravity is also added in this step.

Then the forces from the Springs are computed and added to the Points.

And finally the force from each point is read as an acceleration, used to get the point velocity and update its position.

Resulting in this function:
```
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
}
```
As for how the Spring constraints are handled, first the direction of the force is found, then its amplitude, which is relative to how much the spring is extended or compressed, and is multiplied by how stiff the spring is.

The current speed of the points the spring is attached to is checked, and used along the damping factor to dampen the spring force, simulating energy loses to air resistance, heat and other factors.

This gives this function:
```
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
````
Alongside a very simple handling of ground collisions, this gave me the most basic form of Soft Bodies Simulation, but it was only barely working, and very unstable. The shapes I made kept collapsing, even with multiple layers of very stiff springs, and the points would suffer from extreme rubber-banding when on the ground, generating phantom forces, gaining speed and disappearing from the screen. So the next step was to stabilize all that.

I first clamped the forces generated by the springs, as well as the speed of the points, and as a safety I rounded down to 0 very low forces and speeds.
```
void SoftBody::ClampSpringForce(Spring& spring) {
    Vector3 delta = spring.points[1]->position - spring.points[0]->position;
    float dist = delta.length();

    float minDist = 2.0f * spring.points[0]->radius;
    float maxDist = 2.0f * spring.baseDistance;

    if (dist < minDist || dist > maxDist) {
        delta.normalize();

        float clampedDist = Maths::clamp(dist, minDist, maxDist);
        Vector3 target = spring.points[0]->position + delta * clampedDist;

        // Correction vector (split between points, inversely to mass)
        float totalMass = spring.points[0]->mass + spring.points[1]->mass;
        float ratioA = spring.points[1]->mass / totalMass;
        float ratioB = spring.points[0]->mass / totalMass;

        Vector3 correction = (target - spring.points[1]->position);

        spring.points[0]->position -= correction * ratioA;
        spring.points[1]->position += correction * ratioB;

        // Optional: zero spring force to prevent snapback
        spring.points[0]->force = Vector3();
        spring.points[1]->force = Vector3();
    }
}


void SoftBody::Solve(float dt){

...

const float maxSpeed = 100.0f;
    const float groundFriction=.85f;
    const float lowSpeedThreshold=2.f;
    
    for(int i=0;i<points.size();i++){
        // Zero low force
        if(points[i].force.length()<lowSpeedThreshold)points[i].force=Vector3();
        
        points[i].speed+=points[i].force*dt * (1/points[i].mass);
        
        if (points[i].speed.length() > maxSpeed) {
            points[i].speed.normalize();
            points[i].speed *= maxSpeed;
        }
        
        points[i].position+=points[i].speed*dt;
}
```
This fixed the rubber-banding but not the collasping. This would require either Volume Preservation, which pushes the points away, or pull them towards a virtual center point of the shape depending on if the current volume of the shape is smaller or larger than its original, simulating internal pressure, or Shape Matching, which creates a solid shape from the original position of the points and pull them towards that ideal position during simulation, creating a pseudo frame that moves along with the shape. I decided to go with shape matching as it is similar to what is used in XPBD, given that my future objective is to switch to that method.

### Shape Matching

The implementation of the Shape Matching required quite of bit of Matrix maths and the creation of multiple functions in my maths classes, but in essence its action is fairly simple: it compares the current and original positions of each points to find the most optimal position and rotation of the frame, and then pulls each point towards its pair in the frame, resulting in expansion if the shape is smaller than the frame, and contraction in the opposite case.
```
void SoftBody::ApplyShapeMatching(float stiffness)
{
    if (points.empty() || restPositions.size() != points.size())return;

    // Compute current and rest centers of mass
    Vector3 centerNow=Vector3();
    Vector3 centerRest=Vector3();
    for (size_t i = 0; i < points.size(); ++i) {
        centerNow += points[i].position;
        centerRest += restPositions[i];
    }
    centerNow *= 1.f/static_cast<float>(points.size());
    centerRest *= 1.f/static_cast<float>(restPositions.size());

    // Compute covariance matrix A = Σ(p_i - c_p)(q_i - c_q)^T
    Matrix3 A;
    A.zero();
    for (size_t i = 0; i < points.size(); ++i) {
        Vector3 p = points[i].position - centerNow;
        Vector3 q = restPositions[i] - centerRest;
        A += Matrix3::outerProduct(p, q); // A += pqᵗ
    }

    // Compute optimal rotation matrix
    Matrix3 R = A.orthonormalized();

    // Pull current points toward rotated rest pose
    for (size_t i = 0; i < points.size(); ++i) {
        Vector3 q = restPositions[i] - centerRest;
        Vector3 goal = centerNow + R.getColumn(0) * q.x + R.getColumn(1) * q.y + R.getColumn(2) * q.z;
        Vector3 correction = (goal - points[i].position) * stiffness;
        points[i].position += correction;
    }
}
```

All of this leads to the current, mostly stable simulation and set a good foundation for me to keep working on this project, with two objectives: switching from Euler Integration to Extended Position-Based Dynamics, and implementing that simulation system in a game.

## Bibliography
- [Writing a soft body cube in C/C++ using XPBD | Devlog Episode 1](https://www.youtube.com/watch?v=MgmXJnR62uA) by [blackedout01](https://www.youtube.com/@blackedoutk)
- [Physics of JellyCar: Soft Body Physics Explained](https://www.youtube.com/watch?v=3OmkehAJoyo) by [Walaber Entertainment](https://www.youtube.com/@WalaberEntertainment)
- [Soft Body Physics Explained](https://www.youtube.com/watch?v=kyQP4t_wOGI) by [Gonkee](https://www.youtube.com/@Gonkee)
- [Soft body procedural animation](https://www.youtube.com/watch?v=GXh0Vxg7AnQ) by [argonaut](https://www.youtube.com/@argonautcode)
- [XPBD Extended Position Based Dynamics](https://matthias-research.github.io/pages/tenMinutePhysics/09-xpbd.pdf) by Matthias Müller

- [Havok Physics](https://www.havok.com/havok-physics/) by Havok
- [Bullet3](https://github.com/bulletphysics/bullet3) by BulletPhysics
- [Jelly Engine](https://github.com/Rafapp/jellyengine) by Rafael Padilla
- [Chaos Flesh](https://dev.epicgames.com/documentation/en-us/unreal-engine/chaos-flesh-overview) by Epic Games
- [SoftBody3d](https://docs.godotengine.org/en/stable/classes/class_softbody3d.html) for Godot

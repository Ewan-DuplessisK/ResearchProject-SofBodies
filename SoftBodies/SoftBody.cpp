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

    //Draw();

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



void SoftBody::Solve(float dt){
    for(int i=0;i<points.size();i++){
        if(!pointMassGroundCollisions(&points[i]))points[i].force=Vector3::unitZ*-9*points[i].mass; //This line for gravity
        //points[i].force=Vector3(); //This line for no gravity 
    }
    for(int i=0;i<springs.size();i++){
        SolveSpring(springs[i]);
    }
    for(int i=0;i<springs.size();i++){
        ClampSpringForce(springs[i]);
    }

    ApplyShapeMatching(0.1f);
    
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

        // After movement, apply ground friction *if the point is on the ground*
        auto& groundPlane = getGame().getPlanes()[0];
        const AABB& planeBox = groundPlane->getBox()->getWorldBox();

        // Check if we're within "contact" range (z within 1 unit of planeBox.max.z)
        if (points[i].position.z - 5.0f <= planeBox.max.z + 0.1f) {
            points[i].speed.x *= groundFriction;
            points[i].speed.y *= groundFriction;
        }
    }
    //float tmp = (points[0].position-points[2].position).length();
    //std::cout<<tmp<<std::endl;
}

struct DebugDrawCommand {
    enum class Type { Line, Disc, Hexagon };
    Type type;
    Vector3 start, end; // For lines
    Vector3 center;     // For discs or hexagons
    float radius;
    Vector3 color;
};

const char* vertexShaderSource = R"glsl(
#version 440 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 viewProj;
out vec3 fragColor;
void main() {
    fragColor = aColor;
    gl_Position = viewProj * vec4(aPos, 1.0);
}
)glsl";

const char* fragmentShaderSource = R"glsl(
#version 440 core
in vec3 fragColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(fragColor, 1.0);
}
)glsl";


GLuint CompileShader(GLenum type, const char* source) {
    std::cout<<"start compile"<<std::endl;
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        std::cerr << "Shader compilation failed:\n" << log << std::endl;
    }
    std::cout<<"stop compile"<<std::endl;
    return shader;
}


GLuint CreateShaderProgram(const char* vsSource, const char* fsSource) {
    std::cout<<"start create"<<std::endl;
    GLuint vertex = CompileShader(GL_VERTEX_SHADER, vsSource);
    GLuint fragment = CompileShader(GL_FRAGMENT_SHADER, fsSource);
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program, 512, nullptr, log);
        std::cerr << "Shader linking failed:\n" << log << std::endl;
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);
    std::cout<<"stop create"<<std::endl;
    return program;
}



void SoftBody::Draw(){
    std::vector<DebugDrawCommand> debugCommands;
    
    for (const Point& p : points) {
        debugCommands.push_back({DebugDrawCommand::Type::Hexagon,{}, {}, p.position, 2.0f, Vector3{1.0f, 0.8f, 0.2f}});
    }
    for (const Spring& s : springs) {
        debugCommands.push_back({DebugDrawCommand::Type::Line,s.points[0]->position,s.points[1]->position,{}, 0.0f, Vector3{0.2f, 0.8f, 1.0f}});
    }

    //Render loop
    static GLuint program = CreateShaderProgram(vertexShaderSource, fragmentShaderSource);
    glUseProgram(program);

    // ViewProj
    GLuint viewProjLoc = glGetUniformLocation(program, "viewProj");
    glUniformMatrix4fv(viewProjLoc, 1, GL_FALSE, getGame().getRenderer().getViewProj().getAsFloatPtr());
    
    std::vector<float> lineVertices;
    std::vector<unsigned int> lineIndices;
    std::vector<float> hexVertices;
    std::vector<unsigned int> hexIndices;

    unsigned int lineIndex = 0;
    unsigned int hexIndex = 0;

    const int pointSides = 6;

    for (const DebugDrawCommand& cmd : debugCommands) {
        if (cmd.type == DebugDrawCommand::Type::Line) {
            
            Vector3 color = cmd.color;

            lineVertices.insert(lineVertices.end(), {cmd.start.x, cmd.start.y, cmd.start.z,color.x, color.y, color.z});
            lineVertices.insert(lineVertices.end(), {cmd.end.x, cmd.end.y, cmd.end.z,color.x, color.y, color.z});

            lineIndices.push_back(lineIndex++);
            lineIndices.push_back(lineIndex++);
        }
        else if (cmd.type == DebugDrawCommand::Type::Hexagon) {
            Vector3 color = cmd.color;

            // Center
            hexVertices.insert(hexVertices.end(), {
                cmd.center.x, cmd.center.y, cmd.center.z,
                color.x, color.y, color.z
            });
            unsigned int centerIndex = hexIndex++;

            for (int i = 0; i <= pointSides; ++i) {
                float angle = (float)i / pointSides * 2.0f * M_PI;
                float x = cmd.center.x;
                float y = cmd.center.y + sin(angle) * cmd.radius;
                float z = cmd.center.z + cos(angle) * cmd.radius;

                hexVertices.insert(hexVertices.end(), {
                    x, y, z,
                    color.x, color.y, color.z
                });

                if (i > 0) {
                    hexIndices.push_back(centerIndex);
                    hexIndices.push_back(hexIndex - 1);
                    hexIndices.push_back(hexIndex);
                }
                hexIndex++;
            }
        }
    }

    // ------------------------
    // Render LINES (springs)
    // ------------------------
    GLuint lineVAO, lineVBO, lineEBO;
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);
    glGenBuffers(1, &lineEBO);

    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, lineVertices.size() * sizeof(float), lineVertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lineEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, lineIndices.size() * sizeof(unsigned int), lineIndices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1); // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawElements(GL_LINES, lineIndices.size(), GL_UNSIGNED_INT, nullptr);

    // ------------------------
    // Render HEXAGONS (points)
    // ------------------------
    GLuint hexVAO, hexVBO, hexEBO;
    glGenVertexArrays(1, &hexVAO);
    glGenBuffers(1, &hexVBO);
    glGenBuffers(1, &hexEBO);

    glBindVertexArray(hexVAO);
    glBindBuffer(GL_ARRAY_BUFFER, hexVBO);
    glBufferData(GL_ARRAY_BUFFER, hexVertices.size() * sizeof(float), hexVertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hexEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, hexIndices.size() * sizeof(unsigned int), hexIndices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1); // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawElements(GL_TRIANGLES, hexIndices.size(), GL_UNSIGNED_INT, nullptr);

    // ------------------------
    // Cleanup
    // ------------------------
    glDeleteBuffers(1, &lineVBO);
    glDeleteBuffers(1, &lineEBO);
    glDeleteVertexArrays(1, &lineVAO);

    glDeleteBuffers(1, &hexVBO);
    glDeleteBuffers(1, &hexEBO);
    glDeleteVertexArrays(1, &hexVAO);

}

SoftBody::SoftBody(){

    float arr[3]={60.f,90.f,60.f};
    for(int j=0;j<3;j++){
        for (int i = 0; i < 6; i++) {
            float angle = (float)i / 6 * 2.0f * M_PI;

            points.emplace_back(Point{Vector3{j*60.f,sin(angle)*arr[j],cos(angle)*arr[j]+100.f},Vector3(),Vector3(),2});
        }
    }

    for(int i=0;i<3;i++){
        for(int j=0;j<6;j++){
            Spring s;
            s.points[0]=&points[6*i+j];
            if(j==5)s.points[1]=&points[6*i];
            else s.points[1]=&points[6*i+j+1];
            s.baseDistance=(s.points[0]->position-s.points[1]->position).length();
            s.stiffness=2.f;
            s.damping=5.f;
            springs.emplace_back(s);

            Spring s2;
            s2.points[0]=&points[6*i+j];
            s2.points[1]=&points[6*i+(j+2)%6];
            s2.baseDistance=(s2.points[0]->position-s2.points[1]->position).length();
            s2.stiffness=2.f;
            s2.damping=5.f;
            springs.emplace_back(s2);
        }
    }
    for(int i=0;i<2;i++){
        for(int j=0;j<6;j++){
            Spring s;
            s.points[0]=&points[6*i+j];
            if(j==5)s.points[1]=&points[6*(i+1)];
            else s.points[1]=&points[6*(i+1)+j+1];
            s.baseDistance=(s.points[0]->position-s.points[1]->position).length();
            s.stiffness=2.f;
            s.damping=5.f;
            springs.emplace_back(s);

            Spring s2;
            s2.points[0]=&points[6*i+j];
            s2.points[1]=&points[6*(i+1)+j];
            s2.baseDistance=(s2.points[0]->position-s2.points[1]->position).length();
            s2.stiffness=2.f;
            s2.damping=5.f;
            springs.emplace_back(s2);
        }
    }

    for(Point p:points){
        restPositions.emplace_back(p.position);
    }
    
    //Hexagon
    /*points.emplace_back(Point{Vector3{0,.5f*20.f,3.5f/4.f*20.f+30.f},Vector3(),Vector3(),2});
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
    s.stiffness=2.f;
    s.damping=5.f;
    springs.emplace_back(s);
    Spring s2;
    s2.points[0]=&points[1];
    s2.points[1]=&points[2];
    s2.baseDistance=10.f;
    s2.stiffness=2.f;
    s2.damping=5.f;
    springs.emplace_back(s2);
    Spring s3;
    s3.points[0]=&points[2];
    s3.points[1]=&points[3];
    s3.baseDistance=10.f;
    s3.stiffness=2.f;
    s3.damping=5.f;
    springs.emplace_back(s3);
    Spring s4;
    s4.points[0]=&points[3];
    s4.points[1]=&points[4];
    s4.baseDistance=10.f;
    s4.stiffness=2.f;
    s4.damping=5.f;
    springs.emplace_back(s4);
    Spring s5;
    s5.points[0]=&points[4];
    s5.points[1]=&points[5];
    s5.baseDistance=10.f;
    s5.stiffness=2.f;
    s5.damping=5.f;
    springs.emplace_back(s5);
    Spring s6;
    s6.points[0]=&points[5];
    s6.points[1]=&points[0];
    s6.baseDistance=10.f;
    s6.stiffness=2.f;
    s6.damping=5.f;
    springs.emplace_back(s6);
    
    //Secondary cycle, one-over
    Spring s7;
    s7.points[0]=&points[0];
    s7.points[1]=&points[2];
    s7.baseDistance=25.f;
    s7.stiffness=20.f;
    s7.damping=5.f;
    springs.emplace_back(s7);
    Spring s8;
    s8.points[0]=&points[1];
    s8.points[1]=&points[3];
    s8.baseDistance=25.f;
    s8.stiffness=2.f;
    s8.damping=5.f;
    springs.emplace_back(s8);
    Spring s9;
    s9.points[0]=&points[2];
    s9.points[1]=&points[4];
    s9.baseDistance=25.f;
    s9.stiffness=2.f;
    s9.damping=5.f;
    springs.emplace_back(s9);
    Spring s10;
    s10.points[0]=&points[3];
    s10.points[1]=&points[5];
    s10.baseDistance=25.f;
    s10.stiffness=20.f;
    s10.damping=5.f;
    springs.emplace_back(s10);
    Spring s11;
    s11.points[0]=&points[4];
    s11.points[1]=&points[0];
    s11.baseDistance=25.f;
    s11.stiffness=2.f;
    s11.damping=5.f;
    springs.emplace_back(s11);
    Spring s12;
    s12.points[0]=&points[5];
    s12.points[1]=&points[1];
    s12.baseDistance=25.f;
    s12.stiffness=2.f;
    s12.damping=5.f;
    springs.emplace_back(s12);*/


    //Pyramid
    /*points.emplace_back(Point{Vector3{0,0,50.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{28.f,0.f,30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{-10.f,10.f,30.f},Vector3(),Vector3(),2});
    points.emplace_back(Point{Vector3{-10.f,-10.f,30.f},Vector3(),Vector3(),2});

    Spring s0;
    s0.points[0]=&points[0];
    s0.points[1]=&points[1];
    s0.baseDistance=25.f;
    s0.stiffness=2.f;
    s0.damping=2.f;
    springs.emplace_back(s0);
    Spring s1;
    s1.points[0]=&points[0];
    s1.points[1]=&points[2];
    s1.baseDistance=25.f;
    s1.stiffness=2.f;
    s1.damping=2.f;
    springs.emplace_back(s1);
    Spring s2;
    s2.points[0]=&points[0];
    s2.points[1]=&points[2];
    s2.baseDistance=25.f;
    s2.stiffness=2.f;
    s2.damping=2.f;
    springs.emplace_back(s2);
    Spring s3;
    s3.points[0]=&points[0];
    s3.points[1]=&points[3];
    s3.baseDistance=25.f;
    s3.stiffness=2.f;
    s3.damping=2.f;
    springs.emplace_back(s3);

    Spring s4;
    s4.points[0]=&points[1];
    s4.points[1]=&points[2];
    s4.baseDistance=25.f;
    s4.stiffness=2.f;
    s4.damping=2.f;
    springs.emplace_back(s4);
    Spring s5;
    s5.points[0]=&points[2];
    s5.points[1]=&points[3];
    s5.baseDistance=25.f;
    s5.stiffness=2.f;
    s5.damping=2.f;
    springs.emplace_back(s5);
    Spring s6;
    s6.points[0]=&points[3];
    s6.points[1]=&points[1];
    s6.baseDistance=25.f;
    s6.stiffness=2.f;
    s6.damping=2.f;
    springs.emplace_back(s6);*/
}


#include <raylib.h>
#include <raymath.h>
#include <bullet/btBulletDynamicsCommon.h>

#include <vector>
#include <iostream>

#define FPS 60
#define GRAVITY CLITERAL(Vector3){ 0.0f, -9.81f, 0.0f }

enum class PhysicsType
{
    STATIC,
    DYNAMIC
};

class Object
{
public:
    Object(Vector3 pos, Vector3 rot, Vector3 size, PhysicsType type, float mass, Color color)
    {
        m_ColliderShape = new btBoxShape(
                btVector3(btScalar(size.x/2.0), btScalar(size.y/2.0), btScalar(size.z/2.0)));
        m_Model = LoadModelFromMesh(GenMeshCube(size.x, size.y, size.z));
        m_Color = color;

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(pos.x, pos.y, pos.z));
        transform.setRotation(
                btQuaternion(btScalar(rot.z), btScalar(rot.y), btScalar(rot.x)));

        btScalar objectMass(mass);

        btVector3 localInertia(0, 0, 0);
        if (type == PhysicsType::DYNAMIC && mass != 0.0f)
            m_ColliderShape->calculateLocalInertia(mass, localInertia);

        btDefaultMotionState* motionState = new btDefaultMotionState(transform);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(objectMass, motionState, m_ColliderShape, localInertia);
        m_Body = new btRigidBody(rbInfo);
    }

    ~Object()
    {
        delete m_Body;
        delete m_ColliderShape;
    }

    void Unload(void)
    {
        UnloadModel(m_Model);
    }

    void Render(void)
    {
        if (!m_Body || !m_Body->getMotionState()) {
            return;
        }

        btTransform trans;
        m_Body->getMotionState()->getWorldTransform(trans);

        Vector3 pos = {
            trans.getOrigin().getX(),
            trans.getOrigin().getY(),
            trans.getOrigin().getZ()
        };

        btQuaternion rot = trans.getRotation();
        Vector3 axis = {
            rot.getAxis().getX(),
            rot.getAxis().getY(),
            rot.getAxis().getZ()
        };
        float angle = rot.getAngle() * RAD2DEG;

        DrawModelEx(m_Model, pos, axis, angle, Vector3One(), m_Color);

        Color borderColor = {
            (unsigned char)(m_Color.r/2),
            (unsigned char)(m_Color.g/2),
            (unsigned char)(m_Color.b/2),
            m_Color.a
        };
        DrawModelWiresEx(m_Model, pos, axis, angle, Vector3One(), borderColor);
    }

    btRigidBody* GetRigidBody(void) {
        return m_Body;
    }

private:
    btRigidBody* m_Body;
    btCollisionShape* m_ColliderShape;
    Model m_Model;
    Color m_Color;
};

class Physics
{
public:
    Physics(Vector3 gravity)
    {
        m_CollisionConfiguration = new btDefaultCollisionConfiguration();
        m_Dispatcher = new btCollisionDispatcher(m_CollisionConfiguration);
        m_OverlappingPairCache = new btDbvtBroadphase();
        m_Solver = new btSequentialImpulseConstraintSolver;
        m_DynamicsWorld = new btDiscreteDynamicsWorld(m_Dispatcher, m_OverlappingPairCache, m_Solver, m_CollisionConfiguration);

        m_DynamicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
    }

    ~Physics()
    {
        delete m_CollisionConfiguration;
        delete m_Dispatcher;
        delete m_OverlappingPairCache;
        delete m_Solver;
    }

    void Step(float time)
    {
        m_DynamicsWorld->stepSimulation(time, 10);
    }

    void AddObject(Object& object)
    {
        m_DynamicsWorld->addRigidBody(object.GetRigidBody());
    }

    void RemoveObject(Object& object)
    {
        m_DynamicsWorld->removeRigidBody(object.GetRigidBody());
    }
private:
    btDefaultCollisionConfiguration* m_CollisionConfiguration;
    btCollisionDispatcher* m_Dispatcher;
    btBroadphaseInterface* m_OverlappingPairCache;
    btSequentialImpulseConstraintSolver* m_Solver;
    btDiscreteDynamicsWorld* m_DynamicsWorld;
};

int main(void)
{
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(800, 600, "Randice - Virtual Dice thrower");
    SetTargetFPS(FPS);
    DisableCursor();

    Camera3D camera;
    camera.position = { 30.0f, 30.0f, 30.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 70.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Object ground(
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 10.0f * DEG2RAD, 0.0f },
        { 100.0f, 1.0f, 100.0f },
        PhysicsType::STATIC,
        0.0f,
        GRAY
    );

    std::vector<Object*> cubes;
    const int CUBES = 694;

    Physics physics(GRAVITY);
    physics.AddObject(ground);

    const Color COLORS[25] = {
        LIGHTGRAY, GRAY, DARKGRAY,
        YELLOW, GOLD, ORANGE,
        PINK, RED, MAROON,
        GREEN, LIME, DARKGREEN,
        SKYBLUE, BLUE, DARKBLUE,
        PURPLE, VIOLET, DARKPURPLE,
        BEIGE, BROWN, DARKBROWN,
        WHITE, BLACK, BLANK,
        MAGENTA,
    };

    for (int i = 0; i < CUBES; ++i) {
        Vector3 randomPos = {
            (float)GetRandomValue(-30, 30),
            (float)GetRandomValue(3, 50),
            (float)GetRandomValue(-30, 30)
        };

        Vector3 randomRot = {
            (float)GetRandomValue(0, 90) * DEG2RAD,
            (float)GetRandomValue(0, 90) * DEG2RAD,
            (float)GetRandomValue(0, 90) * DEG2RAD
        };

        Color randomColor = COLORS[GetRandomValue(0, 24)];

        Object* cube = new Object(
            randomPos,
            randomRot,
            { 2.0f, 2.0f, 2.0f },
            PhysicsType::DYNAMIC,
            1.0f,
            randomColor
        );

        cubes.push_back(cube);
        physics.AddObject(*cube);
    }

    float dt = 1.0f;

    while (!WindowShouldClose()) {
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        physics.Step(1.0f / FPS * dt * 100.0f);

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                ground.Render();
                for (Object* cube : cubes) {
                    cube->Render();
                }
            EndMode3D();
        EndDrawing();

        dt = GetFrameTime();
    }

    for (int i = 0; i < CUBES; ++i) {
        physics.RemoveObject(*cubes[i]);
        cubes[i]->Unload();
        delete cubes[i];
        cubes[i] = NULL;
    }

    CloseWindow();

    return 0;
}

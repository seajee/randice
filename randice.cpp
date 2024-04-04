#include <raylib.h>
#include <raymath.h>
#include <bullet/btBulletDynamicsCommon.h>

#include <vector>
#include <memory>

#define FPS 60
#define GRAVITY CLITERAL(Vector3){ 0.0f, -9.81f, 0.0f }
#define CUBES 1
#define OBJECT_BUONCINESS 0.8f

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
        m_ColliderShape = std::make_shared<btBoxShape>(
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

        m_MotionState = std::make_shared<btDefaultMotionState>(transform);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(objectMass, m_MotionState.get(), m_ColliderShape.get(), localInertia);
        rbInfo.m_restitution = OBJECT_BUONCINESS;

        m_Body = std::make_shared<btRigidBody>(rbInfo);
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
        return m_Body.get();
    }

private:
    std::shared_ptr<btRigidBody> m_Body;
    std::shared_ptr<btCollisionShape> m_ColliderShape;
    std::shared_ptr<btDefaultMotionState> m_MotionState;
    Model m_Model;
    Color m_Color;
};

class Physics
{
public:
    Physics(Vector3 gravity)
    {
        m_CollisionConfiguration = std::make_shared<btDefaultCollisionConfiguration>();
        m_Dispatcher = std::make_shared<btCollisionDispatcher>(m_CollisionConfiguration.get());
        m_OverlappingPairCache = std::make_shared<btDbvtBroadphase>();
        m_Solver = std::make_shared<btSequentialImpulseConstraintSolver>();
        m_DynamicsWorld = std::make_shared<btDiscreteDynamicsWorld>(
                m_Dispatcher.get(), m_OverlappingPairCache.get(), m_Solver.get(), m_CollisionConfiguration.get());

        m_DynamicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
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
    std::shared_ptr<btDefaultCollisionConfiguration> m_CollisionConfiguration;
    std::shared_ptr<btCollisionDispatcher> m_Dispatcher;
    std::shared_ptr<btBroadphaseInterface> m_OverlappingPairCache;
    std::shared_ptr<btSequentialImpulseConstraintSolver> m_Solver;
    std::shared_ptr<btDiscreteDynamicsWorld> m_DynamicsWorld;
};

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

int main(void)
{
    // Initialize raylib
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(800, 600, "Randice - Virtual Dice thrower");
    SetTargetFPS(FPS);
    DisableCursor();

    // Setup objects
    Camera3D camera;
    camera.position = { 10.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 70.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Object ground(
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f },
        { 100.0f, 1.0f, 100.0f },
        PhysicsType::STATIC,
        0.0f,
        GRAY
    );

    std::vector<std::shared_ptr<Object>> cubes;
    Vector3 cubeSize = { 2.0f, 2.0f, 2.0f };

    Physics physics(GRAVITY);
    physics.AddObject(ground);

    for (int i = 0; i < CUBES; ++i) {
        // Vector3 randomPos = {
        //     (float)GetRandomValue(-30, 30),
        //     (float)GetRandomValue(3, 50),
        //     (float)GetRandomValue(-30, 30)
        // };

        Vector3 randomPos = { 0.0f, 10.0f, 0.0f };

        Vector3 randomRot = {
            (float)GetRandomValue(0, 360) * DEG2RAD,
            (float)GetRandomValue(0, 360) * DEG2RAD,
            (float)GetRandomValue(0, 360) * DEG2RAD
        };

        Color randomColor = COLORS[GetRandomValue(0, 24)];

        std::shared_ptr<Object> cube = std::make_shared<Object>(
            randomPos,
            randomRot,
            cubeSize,
            PhysicsType::DYNAMIC,
            1.0f,
            randomColor
        );

        cubes.push_back(cube);

        physics.AddObject(*cube);
    }

    float dt = 1.0f;

    // Main loop
    while (!WindowShouldClose()) {
        // Physics
        UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        physics.Step(1.0f / FPS * dt * 100.0f);

        // Rendering
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                DrawGrid(1000, 10);
                ground.Render();
                for (auto cube : cubes) {
                    cube->Render();
                }
            EndMode3D();

            DrawFPS(10, 10);
        EndDrawing();

        dt = GetFrameTime();
    }

    // Unload objects
    for (int i = 0; i < CUBES; ++i) {
        physics.RemoveObject(*cubes[i]);
        cubes[i]->Unload();
    }

    // Close raylib window
    CloseWindow();

    return 0;
}

#ifndef _RANDICE_H_
#define _RANDICE_H_

#include <raylib.h>
#include <raymath.h>
#include <bullet/btBulletDynamicsCommon.h>

#include <memory>

#define GRAVITY CLITERAL(Vector3){ 0.0f, -9.81f, 0.0f }
#define OBJECT_BUONCINESS 0.8f

enum class PhysicsType
{
    STATIC,
    DYNAMIC
};

class Object
{
public:
    Object(Vector3 pos, Vector3 rot, Vector3 size, PhysicsType type, float mass, Color color);
    void Unload(void);
    void Render(void);
    btRigidBody* GetRigidBody(void);

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
    Physics(Vector3 gravity);
    void Step(float time);
    void AddObject(Object& object);
    void RemoveObject(Object& object);

private:
    std::shared_ptr<btDefaultCollisionConfiguration> m_CollisionConfiguration;
    std::shared_ptr<btCollisionDispatcher> m_Dispatcher;
    std::shared_ptr<btBroadphaseInterface> m_OverlappingPairCache;
    std::shared_ptr<btSequentialImpulseConstraintSolver> m_Solver;
    std::shared_ptr<btDiscreteDynamicsWorld> m_DynamicsWorld;
};

#endif // _RANDICE_H_

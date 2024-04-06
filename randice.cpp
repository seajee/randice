#include "randice.h"

Object::Object(Vector3 pos, Vector3 rot, Vector3 size, PhysicsType type, float mass, Color color)
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

void Object::Unload(void)
{
    UnloadModel(m_Model);
}

void Object::Render(void)
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

btRigidBody* Object::GetRigidBody(void)
{
    return m_Body.get();
}

Physics::Physics(Vector3 gravity)
{
    m_CollisionConfiguration = std::make_shared<btDefaultCollisionConfiguration>();
    m_Dispatcher = std::make_shared<btCollisionDispatcher>(m_CollisionConfiguration.get());
    m_OverlappingPairCache = std::make_shared<btDbvtBroadphase>();
    m_Solver = std::make_shared<btSequentialImpulseConstraintSolver>();
    m_DynamicsWorld = std::make_shared<btDiscreteDynamicsWorld>(
            m_Dispatcher.get(), m_OverlappingPairCache.get(), m_Solver.get(), m_CollisionConfiguration.get());

    m_DynamicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
}

void Physics::Step(float time)
{
    m_DynamicsWorld->stepSimulation(time, 10);
}

void Physics::AddObject(Object& object)
{
    m_DynamicsWorld->addRigidBody(object.GetRigidBody());
}

void Physics::RemoveObject(Object& object)
{
    m_DynamicsWorld->removeRigidBody(object.GetRigidBody());
}

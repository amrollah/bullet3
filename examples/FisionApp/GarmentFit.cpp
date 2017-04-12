/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <ThirdPartyLibs/Wavefront/tiny_obj_loader.h>
#include "GarmentFit.h"

#include "btBulletDynamicsCommon.h"

#include "../ExtendedTutorials/RigidBodyFromObj.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include"../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct GarmentFit : public CommonRigidBodyBase {
    int m_options;

    GarmentFit(struct GUIHelperInterface *helper, int options)
            : CommonRigidBodyBase(helper),
              m_options(options) {
    }

    virtual ~GarmentFit() {}

    virtual void initPhysics();

    virtual void renderScene();

    void createEmptyDynamicsWorld() {
        m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
        m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

        m_broadphase = new btDbvtBroadphase();

        m_solver = new btSequentialImpulseConstraintSolver;

        m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
        m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
        m_dynamicsWorld->stepSimulation(btScalar(1.)/btScalar(2000.), btScalar(1.)/btScalar(2000.));

        softBodyWorldInfo.m_broadphase = m_broadphase;
        softBodyWorldInfo.m_dispatcher = m_dispatcher;
        softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
        softBodyWorldInfo.m_sparsesdf.Initialize();
    }

    virtual btSoftRigidDynamicsWorld *getSoftDynamicsWorld() {
        ///just make it a btSoftRigidDynamicsWorld please
        ///or we will add type checking
        return (btSoftRigidDynamicsWorld *) m_dynamicsWorld;
    }

    void resetCamera() {
        float dist = 4;
        float pitch = 150;
        float yaw = 35;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
    }
    void createClothPath(const btScalar s, const int numX, const int numY, const int fixed= 1 + 2);
    void createSoftBody(tinyobj::shape_t& shape);
    void LoadRigidBody(const char* relativeFileName, const char* meshFileName);

    btSoftBodyWorldInfo softBodyWorldInfo;
};

void GarmentFit::initPhysics() {
    m_guiHelper->setUpAxis(1);

    createEmptyDynamicsWorld();
    //m_dynamicsWorld->setGravity(btVector3(0,0,0));
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
//
    if (m_dynamicsWorld->getDebugDrawer())
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(
                btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints
                +btIDebugDraw::DBG_DrawAabb
                +btIDebugDraw::DBG_DrawConstraints);

    ///create a few basic rigid bodies
    btBoxShape *groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));


    //groundShape->initializePolyhedralFeatures();
    //btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -50, 0)); // -75

    {
        btScalar mass(0.);
        createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
    }
    btCollisionShape* childShape1 = new btSphereShape(btScalar(1.5));
//    m_guiHelper->createCollisionShapeGraphicsObject(childShape1);
    m_collisionShapes.push_back(childShape1);
    btCollisionShape* childShape2 = new btSphereShape(btScalar(0.5));
//    m_guiHelper->createCollisionShapeGraphicsObject(childShape2);
    m_collisionShapes.push_back(childShape2);

    btTransform groundTransform1;
    groundTransform1.setIdentity();
    float pos[4] = {0, 1.8, 0, 0};
    btVector3 position(pos[0], pos[1], pos[2]);
    groundTransform1.setOrigin(position);
    {
        btScalar mass(1.);
        createRigidBody(mass, groundTransform1, childShape1, btVector4(0, 0, 1, 1));
    }

    btTransform groundTransform2;
    groundTransform2.setIdentity();
    float pos2[4] = {0, 1.8, 0, 0};
    btVector3 position2(pos[0], pos[1], pos[2]);
    groundTransform2.setOrigin(position2);
    {
        btScalar mass(1.);
        createRigidBody(mass, groundTransform2, childShape2, btVector4(0, 1, 0, 1));
    }


    //load our obj mesh
    const char *bodyFileName = "fision//bodyMesh.obj"; //"teddy.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
    char relativeFileName[1024];
    char pathPrefix[1024];
    if (b3ResourcePath::findResourcePath(bodyFileName, relativeFileName, 1024)) {
        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
    }

    LoadRigidBody(relativeFileName, bodyFileName);

    // create garments
    {
        const btScalar s = 1; //size of cloth patch
        const int NUM_X = 50; //vertices on X axis
        const int NUM_Y = 50; //vertices on Z axis
//        createClothPath(s,NUM_X,NUM_Y);

        const char *jacketFileName = "fision//JacketMesh.obj";
        char relativeFileName2[1024];
        char pathPrefix2[1024];
        if (b3ResourcePath::findResourcePath(jacketFileName, relativeFileName2, 1024)) {
            b3FileUtils::extractPath(relativeFileName2, pathPrefix2, 1024);
        }

        std::vector<tinyobj::shape_t> shapes_j;
        std:: string err = tinyobj::LoadObj(shapes_j, relativeFileName2, pathPrefix2);
        tinyobj::shape_t& jacket = shapes_j[0];

//        createSoftBody(jacket);
//        LoadRigidBody(relativeFileName2, jacketFileName);

        const char *trousersFileName = "fision//TrousersMesh.obj";
        char relativeFileName3[1024];
        char pathPrefix3[1024];
        if (b3ResourcePath::findResourcePath(trousersFileName, relativeFileName3, 1024)) {
            b3FileUtils::extractPath(relativeFileName3, pathPrefix3, 1024);
        }

        std::vector<tinyobj::shape_t> shapes_t;
        err = tinyobj::LoadObj(shapes_t, relativeFileName3, pathPrefix3);
        tinyobj::shape_t& trousers = shapes_t[0];

//        createSoftBody(trousers);

    }

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

}


void GarmentFit::LoadRigidBody(const char* relativeFileName, const char* meshFileName){
    GLInstanceGraphicsShape *glmesh = LoadMeshFromObj(relativeFileName, "");
    printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, meshFileName);

    const GLInstanceVertex &v = glmesh->m_vertices->at(0);
    btConvexHullShape *shape = new btConvexHullShape((const btScalar *) (&(v.xyzw[0])), glmesh->m_numvertices,
                                                     sizeof(GLInstanceVertex));

    float scaling[4] = {1, 1, 1, 1};
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    shape->setLocalScaling(localScaling);

    if (m_options & OptimizeConvexObj) {
        shape->optimizeConvexHull();
    }

//    if (m_options & ComputePolyhedralFeatures) {
//        shape->initializePolyhedralFeatures();
//    }

    shape->setMargin(0.001);
    m_collisionShapes.push_back(shape);

    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(0.f);
    if (meshFileName != "fision//bodyMesh.obj")
        mass = 0.1f;

    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);
    float color[4] = {1, 1, 1, 1};
    if (meshFileName != "fision//bodyMesh.obj")
        color[0] = 0;
    float orn[4] = {0, 0, 0, 1};
    float pos[4] = {0, 0.8, 0, 0};
    btVector3 position(pos[0], pos[1], pos[2]);
    startTransform.setOrigin(position);
    btRigidBody *body = createRigidBody(mass, startTransform, shape);

    body->setContactStiffnessAndDamping(300,10);

    bool useConvexHullForRendering = ((m_options & ObjUseConvexHullForRendering) != 0);


    if (!useConvexHullForRendering) {
        int shapeId = m_guiHelper->registerGraphicsShape(&glmesh->m_vertices->at(0).xyzw[0],
                                                         glmesh->m_numvertices,
                                                         &glmesh->m_indices->at(0),
                                                         glmesh->m_numIndices,
                                                         B3_GL_TRIANGLES, -1);
        shape->setUserIndex(shapeId);
        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
        body->setUserIndex(renderInstance);
    }
}

void GarmentFit::createClothPath(const btScalar s,
                                const int numX,
                                const int numY,
                                const int fixed) {


    btSoftBody *cloth = btSoftBodyHelpers::CreatePatch(softBodyWorldInfo,
                                                       btVector3(-s / 2, s + 1, 0),
                                                       btVector3(+s / 2, s + 1, 0),
                                                       btVector3(-s / 2, s + 1, +s),
                                                       btVector3(+s / 2, s + 1, +s),
                                                       numX, numY,
                                                       fixed, true);
    // This is thickness for cloth
//    cloth->getCollisionShape()->setMargin(0.0001f);
    cloth->generateBendingConstraints(2, cloth->appendMaterial());
    cloth->setTotalMass(0.01);
    //cloth->m_cfg.citerations = 10;
//	cloth->m_cfg.diterations = 10;
    cloth->m_cfg.piterations = 5;
    cloth->m_cfg.kDP = 0.005f;
    getSoftDynamicsWorld()->addSoftBody(cloth);
}


void GarmentFit::createSoftBody(tinyobj::shape_t& objshape) {

    int indices[objshape.mesh.indices.size()];
    for (int i=0;i<objshape.mesh.indices.size();i++)
        indices[i] = (int)objshape.mesh.indices[i];
    btScalar vertices[objshape.mesh.positions.size()];
    for (int i=0;i<objshape.mesh.positions.size();i++)
        vertices[i] = (btScalar)objshape.mesh.positions[i];
    btSoftBody *garment = btSoftBodyHelpers::CreateFromTriMesh(softBodyWorldInfo, &vertices[0],
                                                               &indices[0], (int)objshape.mesh.indices.size()/3, false);
    garment->getCollisionShape()->setMargin(0.001f);
    garment->generateBendingConstraints(2, garment->appendMaterial());
    garment->setTotalMass(0.1);
    garment->m_cfg.piterations = 2;
    garment->m_cfg.kDP = 0.0f;
    garment->m_cfg.kDF = 0.9f;
    getSoftDynamicsWorld()->addSoftBody(garment);
}


void GarmentFit::renderScene() {
    btSoftRigidDynamicsWorld *softWorld = getSoftDynamicsWorld();
    for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++) {
        btSoftBody *psb = (btSoftBody *) softWorld->getSoftBodyArray()[i];
//        if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
        {
            btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
            btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
        }
    }
    CommonRigidBodyBase::renderScene();
}


CommonExampleInterface *BasicExampleCreateFunc(CommonExampleOptions &options) {
    return new GarmentFit(options.m_guiHelper, options.m_option);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)




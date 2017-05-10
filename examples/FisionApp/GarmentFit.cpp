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
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#include <dirent.h>
#include <boost/algorithm/string/predicate.hpp>
#include <string>
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5
#define SCALE_FACTOR 20
#define FIXED_SUBSTEP 1./30.
#define BT_OVERRIDE

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct GarmentFit : public CommonRigidBodyBase {
    int m_options;

    GarmentFit(struct GUIHelperInterface *helper, int options)
            : CommonRigidBodyBase(helper),
              m_options(options) {
        const char *jacketFileName = "fision//10000_vertices//JacketMesh.obj";
        char relativeFileName2[1024];
        char pathPrefix2[1024];
        if (b3ResourcePath::findResourcePath(jacketFileName, relativeFileName2, 1024)) {
            b3FileUtils::extractPath(relativeFileName2, pathPrefix2, 1024);
        }

        std::vector<tinyobj::shape_t> shapes_j;
        std:: string err = tinyobj::LoadObj(shapes_j, relativeFileName2, pathPrefix2);
        jacket = shapes_j[0];

        const char *bodyPartsFileName = "fision//3000_vertices//parts//";
        char relativePartsFileName[1024];
        char pathPartsPrefix[1024];
        if (b3ResourcePath::findResourcePath(bodyPartsFileName, relativePartsFileName, 1024)) {
            b3FileUtils::extractPath(relativePartsFileName, pathPartsPrefix, 1024);
        }
        LoadCompoundBody(relativePartsFileName);
        iteration = 0;
        expansionFactor = 0.01;

        m_guiHelper->setUpAxis(1);

        createEmptyDynamicsWorld();
        m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    }

    virtual ~GarmentFit() {}

    virtual void initPhysics();

    virtual void renderScene();
    virtual void stepSimulation(float deltaTime);

    void createEmptyDynamicsWorld() {
        m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
        m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
        m_broadphase = new btDbvtBroadphase();

        m_solver = new btSequentialImpulseConstraintSolver;

        m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
        m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
//        btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
//        btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

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
        float dist = SCALE_FACTOR * 1.6;
        float pitch = 150;
        float yaw = 35;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
    }

    void createClothPath(const btScalar s, const int numX, const int numY, const int fixed= 0);
    void createSoftBody(tinyobj::shape_t& objshape, bool anchorToBody = false);
    void LoadRigidBody(const char* relativeFileName, const char* meshFileName);
    void LoadCompoundBody(const char* dirPath);
    void createCompoundHumanBody();
    void findFixNodes(btScalar *vertices, std::vector<int> &fixNodes, int size);
    void expandBody(float expansionFactor, btRigidBody* humanBody);
    void setOrigBodyShape();

    btSoftBodyWorldInfo softBodyWorldInfo;
    btRigidBody *body;
    btCompoundShape* compound;
    btRigidBody *compoundBody = NULL;
    GLInstanceGraphicsShape *m_body;
    GLInstanceGraphicsShape *orig_body;
    float expansionFactor;
    int iteration;
    btCollisionShape* body_shape;
    btCollisionShape* orig_human_shape;
    tinyobj::shape_t jacket;
    btSoftBody *garment = NULL;
};

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static float waveheight = 0.5f;
const float TRIANGLE_SIZE=0.5f;

void GarmentFit::stepSimulation(float deltaTime) BT_OVERRIDE {
//    printf("%f\n", deltaTime);
    if (m_dynamicsWorld){
        m_dynamicsWorld->stepSimulation(FIXED_SUBSTEP, 1, FIXED_SUBSTEP);
    }
}

void GarmentFit::initPhysics() {

//
//    if (m_dynamicsWorld->getDebugDrawer())
//        m_dynamicsWorld->getDebugDrawer()->setDebugMode(
//                btIDebugDraw::DBG_DrawWireframe
//                + btIDebugDraw::DBG_DrawContactPoints
//                +btIDebugDraw::DBG_DrawAabb
//                +btIDebugDraw::DBG_DrawConstraints
// );

    ///create ground rigid body
//    btBoxShape *groundShape = createBoxShape(btVector3(btScalar(250.), btScalar(.1), btScalar(250.)));
//
//
//    groundShape->initializePolyhedralFeatures();
////    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
//
//    m_collisionShapes.push_back(groundShape);
////
//    btTransform groundTransform;
//    groundTransform.setIdentity();
//    groundTransform.setOrigin(btVector3(-20, -50, 20)); // -75
//
//    {
//        btScalar mass(0.);
//        createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
//    }



//






    //load our obj mesh
//    const char *orig_bodyFileName = "fision//3000_vertices//orig_bodyMesh.obj"; //bodyMesh.obj "teddy.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
//    char orig_relativeFileName[1024];
//    char orig_pathPrefix[1024];
//    if (b3ResourcePath::findResourcePath(orig_bodyFileName, orig_relativeFileName, 1024)) {
//        b3FileUtils::extractPath(orig_relativeFileName, orig_pathPrefix, 1024);
//    }
//    orig_body = LoadMeshFromObj(orig_relativeFileName, "");
//    setOrigBodyShape();
//    LoadRigidBody(orig_relativeFileName, orig_bodyFileName);

//    const char *bodyFileName = "fision//3000_vertices//bodyMesh.obj"; //bodyMesh_shrink.obj "teddy.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
//    char relativeFileName[1024];
//    char pathPrefix[1024];
//    if (b3ResourcePath::findResourcePath(bodyFileName, relativeFileName, 1024)) {
//        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
//    }

//    LoadRigidBody(relativeFileName, bodyFileName);

//    const char *bodyPartsFileName = "fision//3000_vertices//parts//";
//    char relativePartsFileName[1024];
//    char pathPartsPrefix[1024];
//    if (b3ResourcePath::findResourcePath(bodyPartsFileName, relativePartsFileName, 1024)) {
//        b3FileUtils::extractPath(relativePartsFileName, pathPartsPrefix, 1024);
//    }
//    LoadCompoundBody(relativePartsFileName);
    createCompoundHumanBody();
    // create garments
    {
//        const btScalar s = 5; //size of cloth patch
//        const int NUM_X = 50; //vertices on X axis
//        const int NUM_Y = 50; //vertices on Z axis
//        createClothPath(s,NUM_X,NUM_Y);

//        const char *jacketFileName = "fision//3000_vertices//JacketMesh.obj";
//        char relativeFileName2[1024];
//        char pathPrefix2[1024];
//        if (b3ResourcePath::findResourcePath(jacketFileName, relativeFileName2, 1024)) {
//            b3FileUtils::extractPath(relativeFileName2, pathPrefix2, 1024);
//        }
//
//        std::vector<tinyobj::shape_t> shapes_j;
//        std:: string err = tinyobj::LoadObj(shapes_j, relativeFileName2, pathPrefix2);
//        tinyobj::shape_t& jacket = shapes_j[0];
        if (!garment)
            createSoftBody(jacket);

//        const char *trousersFileName = "fision//3000_vertices//TrousersMesh.obj";
//        char relativeFileName3[1024];
//        char pathPrefix3[1024];
//        if (b3ResourcePath::findResourcePath(trousersFileName, relativeFileName3, 1024)) {
//            b3FileUtils::extractPath(relativeFileName3, pathPrefix3, 1024);
//        }
//
//        std::vector<tinyobj::shape_t> shapes_t;
//        err = tinyobj::LoadObj(shapes_t, relativeFileName3, pathPrefix3);
//        tinyobj::shape_t& trousers = shapes_t[0];

//        createSoftBody(trousers, true);

    }
    btVector3 ilocalScaling = compound->getLocalScaling();
    printf("scale: %f\n",ilocalScaling.getX());
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

}

/*
 * Useless for now
 */
void GarmentFit::setOrigBodyShape(){
    const GLInstanceVertex &v = orig_body->m_vertices->at(0);
    int indexStride = 3*sizeof(int);
    btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(orig_body->m_numIndices/3,
                                                                                   &orig_body->m_indices->at(0),
                                                                                   indexStride,
                                                                                   orig_body->m_numvertices,(btScalar*) (&(v.xyzw[0])),sizeof(GLInstanceVertex));

//    btGImpactMeshShape * orig_human_shape = new btGImpactMeshShape(indexVertexArrays);
//    orig_human_shape->updateBound();
//
    orig_human_shape = new btBvhTriangleMeshShape(indexVertexArrays,true);

    float scaling[4] = {SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR, 1};
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    orig_human_shape->setLocalScaling(localScaling);
}

void GarmentFit::LoadCompoundBody(const char* dirPath){
    compound = new btCompoundShape();
    m_collisionShapes.push_back (compound);

    btTransform startTransform;
    startTransform.setIdentity();
    float pos[4] = {0, 0, 0, 0};
    btVector3 position(pos[0], pos[1], pos[2]);
    startTransform.setOrigin(position);

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (dirPath)) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            if (boost::ends_with(ent->d_name, ".obj")) {
                char objFile[2000] = "";
                strcat(objFile, dirPath);
                printf("loading %s\n", strcat(objFile, ent->d_name));
                GLInstanceGraphicsShape *bodyPart = LoadMeshFromObj(objFile, "");
                const GLInstanceVertex &v = bodyPart->m_vertices->at(0);
                btConvexHullShape *bodyPart_shape = new btConvexHullShape((const btScalar *) (&(v.xyzw[0])),
                                                                          bodyPart->m_numvertices,
                                                                          sizeof(GLInstanceVertex));

                compound->addChildShape(startTransform, bodyPart_shape);
            }
        }
        closedir(dir);
    } else {
        perror ("could not open directory");
    }

}

void GarmentFit::createCompoundHumanBody(){
    btTransform startTransform;
    startTransform.setIdentity();
    btScalar mass(0.0f);
    float scaling[4] = {SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR, 1}; // just for the first time  we set this scale
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    if (!compoundBody)
        compound->setLocalScaling(localScaling);
    compound->setMargin(0.4f);
    startTransform.setRotation(btQuaternion(-SIMD_PI,0,0));
    compoundBody = createRigidBody(mass, startTransform, compound);
    compoundBody->setFriction( 0.3f );
//    compoundBody->setCollisionFlags(btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT);
//    compoundBody->setActivationState( DISABLE_DEACTIVATION );
}

void GarmentFit::LoadRigidBody(const char* relativeFileName, const char* meshFileName){
    m_body = LoadMeshFromObj(relativeFileName, "");
    printf("[INFO] Obj loaded: Extracted %d vertices from obj file [%s]\n", m_body->m_numvertices, meshFileName);

    const GLInstanceVertex &v = m_body->m_vertices->at(0);
//    btConvexHullShape *shape = new btConvexHullShape((const btScalar *) (&(v.xyzw[0])), m_body->m_numvertices,
//                                                     sizeof(GLInstanceVertex));

    int indexStride = 3*sizeof(int);
    btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(m_body->m_numIndices/3,
                                                                                   &m_body->m_indices->at(0),
                                                                                   indexStride,
                                                                                   m_body->m_numvertices,(btScalar*) (&(v.xyzw[0])),sizeof(GLInstanceVertex));

//    btGImpactMeshShape * body_shape = new btGImpactMeshShape(indexVertexArrays);
//    body_shape->updateBound();
//
//    bool useQuantizedAabbCompression = true;
    body_shape = new btBvhTriangleMeshShape(indexVertexArrays,true);
//    body_shape = new btCapsuleShape(0.2, 0.2);

    float scaling[4] = {SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR, 1};
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    body_shape->setLocalScaling(localScaling);

//    if (m_options & OptimizeConvexObj) {
//        shape->optimizeConvexHull();
//    }
//
//    if (m_options & ComputePolyhedralFeatures) {
//        shape->initializePolyhedralFeatures();
//    }

    body_shape->setMargin(0.5);
//    m_collisionShapes.push_back(shape);

    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(0.0f);
//    if (meshFileName != "fision//bodyMesh.obj")
//        mass = 0.1f;

    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        body_shape->calculateLocalInertia(mass, localInertia);
    float color[4] = {1, 1, 1, 1};
    if (meshFileName != "fision//bodyMesh.obj")
        color[0] = 0;
    float orn[4] = {0, 0, 0, 1};
    float pos[4] = {0, 0, 0, 0};
    btVector3 position(pos[0], pos[1], pos[2]);
    startTransform.setOrigin(position);
    body = createRigidBody(mass, startTransform, body_shape);


    body->setFriction( 0.8f );

//    bool useConvexHullForRendering = ((m_options & ObjUseConvexHullForRendering) != 0);
//    if (!useConvexHullForRendering) {
//        int shapeId = m_guiHelper->registerGraphicsShape(&orig_body->m_vertices->at(0).xyzw[0],
//                                                         orig_body->m_numvertices,
//                                                         &orig_body->m_indices->at(0),
//                                                         orig_body->m_numIndices,
//                                                         B3_GL_TRIANGLES, -1);
//        body_shape->setUserIndex(shapeId);
//        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
//        body->setUserIndex(renderInstance);
//    }
}

void GarmentFit::createClothPath(const btScalar s,
                                const int numX,
                                const int numY,
                                const int fixed) {


    btSoftBody *cloth = btSoftBodyHelpers::CreatePatch(softBodyWorldInfo,
                                                       btVector3(-3*s, SCALE_FACTOR + 2 +s + 0.5, -0.5),
                                                       btVector3(+3*s, SCALE_FACTOR + 2 +s + 0.5, -0.5),
                                                       btVector3(-3*s, SCALE_FACTOR + 2 +s + 1, +s-0.5),
                                                       btVector3(+3*s, SCALE_FACTOR + 2 +s + 1, +s-0.5),
                                                       numX, numY,
                                                       fixed, true);
    // This is thickness for cloth
    cloth->getCollisionShape()->setMargin(0.5f);
    cloth->generateBendingConstraints(2, cloth->appendMaterial());
    cloth->setTotalMass(0.1);
    cloth->m_cfg.citerations = 10;
	cloth->m_cfg.diterations = 10;
    cloth->m_cfg.piterations = 10;
    getSoftDynamicsWorld()->addSoftBody(cloth);
}


void GarmentFit::createSoftBody(tinyobj::shape_t& objshape, bool anchorToBody) {

    int indices[objshape.mesh.indices.size()];
    for (int i=0;i<objshape.mesh.indices.size();i++)
        indices[i] = (int)objshape.mesh.indices[i];
    btScalar vertices[objshape.mesh.positions.size()];
    for (int i=0;i<objshape.mesh.positions.size();i++)
        vertices[i] = (btScalar)objshape.mesh.positions[i];
    garment = btSoftBodyHelpers::CreateFromTriMesh(softBodyWorldInfo, &vertices[0],
                                                               &indices[0], (int)objshape.mesh.indices.size()/3);
    float scaling[4] = {SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR, 1};
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    garment->scale(localScaling);

    float margin = std::max(0.35, SCALE_FACTOR/1500.0);
//    printf("margin: %f\n", margin);
    garment->getCollisionShape()->setMargin(margin);
    btSoftBody::Material*	pm=garment->appendMaterial();
    pm->m_kLST				=	0;
    pm->m_kAST              =   0.0;
    pm->m_kVST              =   0.0;
//    pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
//    garment->generateClusters(1024);
    garment->generateBendingConstraints(2,pm);
    garment->m_cfg.piterations = 30;
    garment->m_cfg.citerations = 10;
    garment->m_cfg.diterations = 10;
    garment->m_cfg.kDF			=	0.0;
    garment->m_cfg.kMT          =   0.0;
    garment->m_cfg.kDP          =  0.0;
//    garment->m_cfg.kLF			=	0.05;
//    garment->m_cfg.kDG			=	0.01;
//    garment->m_cfg.kCHR			=	0.01;

//    garment->m_cfg.kLF			=	0.05;
//    garment->m_cfg.kDG			=	0.01;
//    garment->m_cfg.kSHR			=	1;
//    garment->m_cfg.kMT			=	1;

//    garment->m_cfg.collisions = btSoftBody::fCollision::CL_RS;
//enable cluster collision between soft body and soft body
//    garment->m_cfg.collisions += btSoftBody::fCollision::RVSmask;
//    garment->m_cfg.collisions += btSoftBody::fCollision::SVSmask;

//    garment->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
//            btSoftBody::fCollision::CL_SS+
// 	btSoftBody::fCollision::CL_RS
// +btSoftBody::fCollision::CL_SELF +
//                    btSoftBody::fCollision::SDF_RS
//            ;
//    garment->generateClusters(0);
    garment->randomizeConstraints();
//    garment->scale(btVector3(6,6,6));
//    garment->m_cfg.timescale = 10;
    garment->setTotalMass(10,true);
    if (anchorToBody) {
        std::vector<int> fixNodes;
        for (int i=1,idx=0;i<objshape.mesh.positions.size();i+=3,idx++){
//            printf("%f", vertices[i]);
            if (vertices[i]>(0.01)){ // The waist band is around 0.01 height in the mesh vertices
                fixNodes.push_back(idx);
            }
        }
//        findFixNodes(vertices, fixNodes, objshape.mesh.positions.size());
//    std::vector<int> fixNodes = {0, 40, 50, 100, 110, 140, 1001,202,24, 400 , 500};
//    for (int i=0;i<garment->m_nodes.size(); i+=30)
//        garment->appendAnchor(i,body);
        for (std::vector<int>::iterator it = fixNodes.begin(); it != fixNodes.end(); ++it) {
            garment->appendAnchor(*it, compoundBody);
//            printf("anchor: %d\n", *it);
        }
    }
//    float scale = 0.06;
//    garment->setRestLengthScale(scale);
//    btSoftBody::Face f;
//    for (int i=0;i<garment->m_faces.size();f = garment->m_faces.at(i),i++){
//        f.m_ra *= scale;
//    }

//    garment->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
//    garment->setWindVelocity(btVector3(10, 0, 10));
    //this could help performance in some cases
    btSoftBodyHelpers::ReoptimizeLinkOrder(garment);
    getSoftDynamicsWorld()->addSoftBody(garment);
}

void GarmentFit::findFixNodes(btScalar *vertices, std::vector<int> &fixNodes, int size){
//TODO: implement a method to extract top points or vertex group points to fix later or anchor to body.
    int i,j;
    // find the minimum and maximum of z axis in vertices
    btScalar minX = vertices[0];
    int minXidx = 0;
    btScalar maxX = vertices[0];
    int maxXidx = 0;
    for(i=0,j=0;i<size;++j,i+=3)
    {
        if (vertices[i] < minX){
            minX =  vertices[i];
            minXidx = j;
        }
        if (vertices[i] > maxX){
            maxX =  vertices[i];
            maxXidx = j;
        }
    }
    fixNodes.push_back(minXidx);
    fixNodes.push_back(maxXidx);
}

void GarmentFit::renderScene() {
    CommonRigidBodyBase::renderScene();
    btSoftRigidDynamicsWorld *softWorld = getSoftDynamicsWorld();
    for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++) {
        btSoftBody *psb = (btSoftBody *) softWorld->getSoftBodyArray()[i];
//        if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
        {
//            btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
            btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), fDrawFlags::Faces
//                                                                      + fDrawFlags::Contacts
            );
        }
    }
    if (iteration>=0 &&  iteration%20==0) {
        if (expansionFactor < 1) {
            expansionFactor = std::min(expansionFactor + 0.1, 1.0);
            printf("iteration:: %d \n", iteration);
            expandBody(expansionFactor, compoundBody);
            m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
        }
    }
    iteration++;
}

void GarmentFit::expandBody(float expansionFactor, btRigidBody* humanBody){
    //remove the rigidbodies from the dynamics world and delete them
    btSoftRigidDynamicsWorld *softWorld = getSoftDynamicsWorld();
//    softWorld->removeSoftBody(garment);
//    softWorld->removeRigidBody(humanBody);
//    humanBody->setCollisionFlags(btCollisionObject::CollisionFlags::CF_NO_CONTACT_RESPONSE);
//    softWorld->getPairCache()->cleanProxyFromPairs(humanBody->getBroadphaseProxy(),m_dynamicsWorld->getDispatcher());


//    if (softWorld)
//    {
//
//        int i;
//        for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
//        {
//            m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
//        }
//        for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
//        {
//            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
//            btRigidBody* body = btRigidBody::upcast(obj);
//            if (body && body->getMotionState())
//            {
//                delete body->getMotionState();
//            }
//            m_dynamicsWorld->removeCollisionObject(obj);
//            delete obj;
//        }
//    }
    //delete collision shapes
//    for (int j = 0; j<m_collisionShapes.size(); j++)
//    {
//        btCollisionShape* shape = m_collisionShapes[j];
//        delete shape;
//    }
//    m_collisionShapes.clear();


//    softWorld->removeRigidBody(compoundBody);
//    softWorld->removeCollisionObject(compoundBody);
//    m_dynamicsWorld->removeCollisionObject(compoundBody);

//    for (int i=0;i<m_body->m_numvertices;i++){
//        m_body->m_vertices->at(i).xyzw[0] = (1-expansionFactor)*m_body->m_vertices->at(i).xyzw[0] + expansionFactor*orig_body->m_vertices->at(i).xyzw[0];
//        m_body->m_vertices->at(i).xyzw[1] = (1-expansionFactor)*m_body->m_vertices->at(i).xyzw[1] + expansionFactor*orig_body->m_vertices->at(i).xyzw[1];
//        m_body->m_vertices->at(i).xyzw[2] = (1-expansionFactor)*m_body->m_vertices->at(i).xyzw[2] + expansionFactor*orig_body->m_vertices->at(i).xyzw[2];

//        m_body->m_vertices->at(i).normal[0] = (1-expansionFactor)*m_body->m_vertices->at(i).normal[0] + expansionFactor*orig_body->m_vertices->at(i).normal[0];
//        m_body->m_vertices->at(i).normal[1] = (1-expansionFactor)*m_body->m_vertices->at(i).normal[1] + expansionFactor*orig_body->m_vertices->at(i).normal[1];
//        m_body->m_vertices->at(i).normal[2] = (1-expansionFactor)*m_body->m_vertices->at(i).normal[2] + expansionFactor*orig_body->m_vertices->at(i).normal[2];
//
//        m_body->m_vertices->at(i).uv[0] = (1-expansionFactor)*m_body->m_vertices->at(i).uv[0] + expansionFactor*orig_body->m_vertices->at(i).uv[0];
//        m_body->m_vertices->at(i).uv[1] = (1-expansionFactor)*m_body->m_vertices->at(i).uv[1] + expansionFactor*orig_body->m_vertices->at(i).uv[1];
//    }
//    body_shape = new btCapsuleShape(0.2+expansionFactor, 0.2);
//    const GLInstanceVertex &v = m_body->m_vertices->at(0);
//
//    int indexStride = 3*sizeof(int);
//    btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(m_body->m_numIndices/3,
//                                                                                   &m_body->m_indices->at(0),
//                                                                                   indexStride,
//                                                                                   m_body->m_numvertices,(btScalar*) (&(v.xyzw[0])),sizeof(GLInstanceVertex));

//    btGImpactMeshShape * shape = new btGImpactMeshShape(indexVertexArrays);
//    shape->updateBound();

//    btConvexHullShape *shape = new btConvexHullShape((const btScalar *) (&(v.xyzw[0])), m_body->m_numvertices,
//                                                     sizeof(GLInstanceVertex));
//
//    bool useQuantizedAabbCompression = true;
//    btCollisionShape* shape = new btBvhTriangleMeshShape(indexVertexArrays,true);
//
    float scaling[4] = {SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR, 1};
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    humanBody->getCollisionShape()->setLocalScaling(localScaling*(1-expansionFactor/2));
    m_dynamicsWorld->updateSingleAabb(humanBody);
//    softWorld->addCollisionObject(humanBody);

//    if (m_options & OptimizeConvexObj) {
//        shape->optimizeConvexHull();
//    }

//    if (m_options & ComputePolyhedralFeatures) {
//        shape->initializePolyhedralFeatures();
//    }

//    body_shape->setMargin(0.5);
//    m_collisionShapes.push_back(shape);

    btTransform startTransform;
    startTransform.setIdentity();
//
    btScalar mass(0.0f);
//    if (meshFileName != "fision//bodyMesh.obj")
//        mass = 0.1f;

//    bool isDynamic = (mass != 0.f);
//    btVector3 localInertia(0, 0, 0);
//    if (isDynamic)
//        shape->calculateLocalInertia(mass, localInertia);
    float color[4] = {1, 1, 1, 1};
//    if (meshFileName != "fision//bodyMesh.obj")
//        color[0] = 0;
    float orn[4] = {0, 0, 0, 1};
    float pos[4] = {0, 0, 0, 0};
    btVector3 position(pos[0], pos[1], pos[2]);
    startTransform.setOrigin(position);
    startTransform.setRotation(btQuaternion(-SIMD_PI,0,0));
//    delete compoundBody;
//    compound->setLocalScaling(localScaling*(1-expansionFactor/3));
//    compoundBody->setCollisionShape(compound);
//    humanBody = createRigidBody(mass, startTransform, compound);
//    m_dynamicsWorld->addCollisionObject(humanBody);
//    body = createRigidBody(mass, startTransform, body_shape);
//    softWorld->addRigidBody(body);
//
//    body->setFriction( 0.01f );
//
//        int shapeId = m_guiHelper->registerGraphicsShape(&m_body->m_vertices->at(0).xyzw[0],
//                                                         m_body->m_numvertices,
//                                                         &m_body->m_indices->at(0),
//                                                         m_body->m_numIndices,
//                                                         B3_GL_TRIANGLES, -1);
//        body_shape->setUserIndex(shapeId);
//        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
//        body->setUserIndex(renderInstance);
//    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
//    m_guiHelper->removeAllGraphicsInstances();
//    softWorld->addCollisionObject(humanBody);
//    initPhysics();
}

CommonExampleInterface *BasicExampleCreateFunc(CommonExampleOptions &options) {
    return new GarmentFit(options.m_guiHelper, options.m_option);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)




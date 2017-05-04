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

#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5
#define SCALE_FACTOR 20
#define FIXED_SUBSTEP 1./20.

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
    void findFixNodes(btScalar *vertices, std::vector<int> &fixNodes, int size);
    void expandBody(float expansionFactor);
    void setOrigBodyShape();

    btSoftBodyWorldInfo softBodyWorldInfo;
    btRigidBody *body;
    GLInstanceGraphicsShape *m_body;
    GLInstanceGraphicsShape *orig_body;
    float expansionFactor;
    int iteration;
    btCollisionShape* body_shape;
    btCollisionShape* orig_human_shape;
};

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static float waveheight = 0.5f;
const float TRIANGLE_SIZE=0.5f;

void GarmentFit::stepSimulation(float deltaTime) {
    printf("%f\n", deltaTime);
    if (m_dynamicsWorld){
        m_dynamicsWorld->stepSimulation(FIXED_SUBSTEP, 1, FIXED_SUBSTEP);
    }
}

void GarmentFit::initPhysics() {
    m_guiHelper->setUpAxis(1);
    expansionFactor = 0.01;
    iteration = 0;

    createEmptyDynamicsWorld();
    //m_dynamicsWorld->setGravity(btVector3(0,0,0));
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
//
//    if (m_dynamicsWorld->getDebugDrawer())
//        m_dynamicsWorld->getDebugDrawer()->setDebugMode(
//                btIDebugDraw::DBG_DrawWireframe
////                + btIDebugDraw::DBG_DrawContactPoints
//                +btIDebugDraw::DBG_DrawAabb
////                +btIDebugDraw::DBG_DrawConstraints
// );

//    btCollisionShape* groundShape = 0;
//    {
//        int i;
//        int j;
//
//        const int NUM_VERTS_X = 30;
//        const int NUM_VERTS_Y = 30;
//        const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
//        const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);
//
//        gGroundVertices = new btVector3[totalVerts];
//        gGroundIndices = new int[totalTriangles*3];
//
//        btScalar offset(-50);
//
//        for ( i=0;i<NUM_VERTS_X;i++)
//        {
//            for (j=0;j<NUM_VERTS_Y;j++)
//            {
//                gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
//                        //0.f,
//                                                          waveheight*sinf((float)i)*cosf((float)j+offset),
//                                                          (j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
//            }
//        }
//
//        int vertStride = sizeof(btVector3);
//        int indexStride = 3*sizeof(int);
//
//        int index=0;
//        for ( i=0;i<NUM_VERTS_X-1;i++)
//        {
//            for (j=0;j<NUM_VERTS_Y-1;j++)
//            {
//                gGroundIndices[index++] = j*NUM_VERTS_X+i;
//                gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
//                gGroundIndices[index++] = j*NUM_VERTS_X+i+1;;
//
//                gGroundIndices[index++] = j*NUM_VERTS_X+i;
//                gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
//                gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
//            }
//        }
//
//        btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
//                                                                                       gGroundIndices,
//                                                                                       indexStride,
//                                                                                       totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);
//
//        bool useQuantizedAabbCompression = true;
//
//        groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
//        groundShape->setMargin(0.5);
//    }
//
//    m_collisionShapes.push_back(groundShape);
//
//    btTransform groundTransform;
//    groundTransform.setIdentity();
//    groundTransform.setOrigin(btVector3(0, -1.22, 0)); // -75
//
//    {
//        btScalar mass(0.);
//        createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
//    }

    ///create ground rigid body
//    btBoxShape *groundShape = createBoxShape(btVector3(btScalar(250.), btScalar(.1), btScalar(250.)));
//
//
//    groundShape->initializePolyhedralFeatures();
////    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
//
//    m_collisionShapes.push_back(groundShape);
//
//    btTransform groundTransform;
//    groundTransform.setIdentity();
//    groundTransform.setOrigin(btVector3(-20, -50, 20)); // -75
//
//    {
//        btScalar mass(0.);
//        createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
//    }

    // Create a Sphere object
//    btTransform startTransform;
//    startTransform.setIdentity();
//    startTransform.setOrigin(btVector3(0.15,1.8,0));
////    startTransform.setRotation(btQuaternion(SIMD_PI/2,0,0));
//    btCollisionShape* capsuleShape= new btCapsuleShape(1, 2);
//    capsuleShape->setMargin( 0.5 );
//    btRigidBody*		body=createRigidBody(0,startTransform,capsuleShape);
//    body->setFriction( 0.8f );
//    m_collisionShapes.push_back(capsuleShape);

//    btTransform startTransform2;
//    startTransform2.setIdentity();
//    startTransform2.setOrigin(btVector3(0,1,-0.4));
//    startTransform2.setRotation(btQuaternion(0,SIMD_PI/4,SIMD_PI/4));
//    btCollisionShape* capsuleShape2= new btCapsuleShape(0.15, 1);
//    capsuleShape2->setMargin( 0.1 );
//    m_collisionShapes.push_back(capsuleShape2);
//    btRigidBody*		arm1=createRigidBody(0,startTransform2,capsuleShape2);
//    arm1->setFriction( 0.8f );
//
//    btTransform startTransform3;
//    startTransform3.setIdentity();
//    startTransform3.setOrigin(btVector3(0,1,0.4));
//    startTransform3.setRotation(btQuaternion(0,-SIMD_PI/4,SIMD_PI/4));
//    btCollisionShape* capsuleShape3= new btCapsuleShape(0.15, 1);
//    capsuleShape3->setMargin( 0.1 );
//    m_collisionShapes.push_back(capsuleShape3);
//    btRigidBody*		arm2=createRigidBody(0,startTransform3,capsuleShape3);
//    arm2->setFriction( 0.8f );



//
//    btCollisionShape* childShape1 = new btSphereShape(btScalar(1.5));
//
//    btTransform groundTransform1;
//    groundTransform1.setIdentity();
//    float pos[4] = {0, 1.8, 0, 0};
//    btVector3 position(pos[0], pos[1], pos[2]);
//    groundTransform1.setOrigin(position);
//    {
//        btScalar mass(1.);
//        btRigidBody *obj1 = createRigidBody(mass, groundTransform1, childShape1, btVector4(0, 0, 1, 1));
//        getSoftDynamicsWorld()->addRigidBody(obj1);
//    }
//
//    btTransform groundTransform2;
//    groundTransform2.setIdentity();
//    float pos2[4] = {0, 1.8, 0, 0};
//    btVector3 position2(pos[0], pos[1], pos[2]);
//    groundTransform2.setOrigin(position2);
//    {
//        btScalar mass(1.);
//        btRigidBody *obj2 = createRigidBody(mass, groundTransform2, childShape2, btVector4(0, 1, 0, 1));
//        getSoftDynamicsWorld()->addRigidBody(obj2);
//    }

    //load our obj mesh
    const char *orig_bodyFileName = "fision//3000_vertices//orig_bodyMesh.obj"; //bodyMesh.obj "teddy.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
    char orig_relativeFileName[1024];
    char orig_pathPrefix[1024];
    if (b3ResourcePath::findResourcePath(orig_bodyFileName, orig_relativeFileName, 1024)) {
        b3FileUtils::extractPath(orig_relativeFileName, orig_pathPrefix, 1024);
    }
    orig_body = LoadMeshFromObj(orig_relativeFileName, "");
//    setOrigBodyShape();
//    LoadRigidBody(orig_relativeFileName, orig_bodyFileName);

    const char *bodyFileName = "fision//3000_vertices//bodyMesh.obj"; //bodyMesh_shrink.obj "teddy.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
    char relativeFileName[1024];
    char pathPrefix[1024];
    if (b3ResourcePath::findResourcePath(bodyFileName, relativeFileName, 1024)) {
        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
    }

    LoadRigidBody(relativeFileName, bodyFileName);

    // create garments
    {
        const btScalar s = 5; //size of cloth patch
        const int NUM_X = 50; //vertices on X axis
        const int NUM_Y = 50; //vertices on Z axis
//        createClothPath(s,NUM_X,NUM_Y);

        const char *jacketFileName = "fision//3000_vertices//JacketMesh.obj";
        char relativeFileName2[1024];
        char pathPrefix2[1024];
        if (b3ResourcePath::findResourcePath(jacketFileName, relativeFileName2, 1024)) {
            b3FileUtils::extractPath(relativeFileName2, pathPrefix2, 1024);
        }

        std::vector<tinyobj::shape_t> shapes_j;
        std:: string err = tinyobj::LoadObj(shapes_j, relativeFileName2, pathPrefix2);
        tinyobj::shape_t& jacket = shapes_j[0];

        createSoftBody(jacket);

        const char *trousersFileName = "fision//3000_vertices//TrousersMesh.obj";
        char relativeFileName3[1024];
        char pathPrefix3[1024];
        if (b3ResourcePath::findResourcePath(trousersFileName, relativeFileName3, 1024)) {
            b3FileUtils::extractPath(relativeFileName3, pathPrefix3, 1024);
        }

        std::vector<tinyobj::shape_t> shapes_t;
        err = tinyobj::LoadObj(shapes_t, relativeFileName3, pathPrefix3);
        tinyobj::shape_t& trousers = shapes_t[0];

        createSoftBody(trousers, true);

    }

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
//    body_shape = new btCapsuleShape(0.01, 0.2);

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

    bool useConvexHullForRendering = ((m_options & ObjUseConvexHullForRendering) != 0);
    if (!useConvexHullForRendering) {
        int shapeId = m_guiHelper->registerGraphicsShape(&orig_body->m_vertices->at(0).xyzw[0],
                                                         orig_body->m_numvertices,
                                                         &orig_body->m_indices->at(0),
                                                         orig_body->m_numIndices,
                                                         B3_GL_TRIANGLES, -1);
        body_shape->setUserIndex(shapeId);
        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
        body->setUserIndex(renderInstance);
    }
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
    btSoftBody *garment = btSoftBodyHelpers::CreateFromTriMesh(softBodyWorldInfo, &vertices[0],
                                                               &indices[0], (int)objshape.mesh.indices.size()/3);
    float scaling[4] = {SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR, 1};
    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
    garment->scale(localScaling);

    float margin = std::max(0.15, SCALE_FACTOR/1500.0);
    printf("margin: %f\n", margin);
    garment->getCollisionShape()->setMargin(margin);
    btSoftBody::Material*	pm=garment->appendMaterial();
    pm->m_kLST				=	0;
    pm->m_kAST              =   0.0;
    pm->m_kVST              =   0.0;
//    pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
//    garment->generateClusters(1024);
    garment->generateBendingConstraints(2,pm);
    garment->m_cfg.piterations = 50;
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
    garment->setTotalMass(1,true);
    if (anchorToBody) {
        std::vector<int> fixNodes;
        for (int i=1,idx=0;i<objshape.mesh.positions.size();i+=3,idx++){
            printf("%f", vertices[i]);
            if (vertices[i]>(0.01)){ // The waist band is around 0.01 height in the mesh vertices
                fixNodes.push_back(idx);
            }
        }
//        findFixNodes(vertices, fixNodes, objshape.mesh.positions.size());
//    std::vector<int> fixNodes = {0, 40, 50, 100, 110, 140, 1001,202,24, 400 , 500};
//    for (int i=0;i<garment->m_nodes.size(); i+=30)
//        garment->appendAnchor(i,body);
        for (std::vector<int>::iterator it = fixNodes.begin(); it != fixNodes.end(); ++it) {
            garment->appendAnchor(*it, body);
            printf("anchor: %d\n", *it);
        }
    }
//    float scale = 0.06;
//    garment->setRestLengthScale(scale);
//    btSoftBody::Face f;
//    for (int i=0;i<garment->m_faces.size();f = garment->m_faces.at(i),i++){
//        f.m_ra *= scale;
//    }

//    garment->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
//    garment->setWindVelocity(btVector3(0, -14.0, 0));
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
    iteration++;
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
    if (iteration>0 &&  iteration%10==0) {
        if (expansionFactor < 1) {
            expansionFactor = std::min(expansionFactor + 0.01, 1.0);
//            printf("iteration:: %d \n", iteration);
            expandBody(expansionFactor);
//            m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
        }
    }
}

void GarmentFit::expandBody(float expansionFactor){
    //remove the rigidbodies from the dynamics world and delete them
    btSoftRigidDynamicsWorld *softWorld = getSoftDynamicsWorld();
    softWorld->removeRigidBody(body);

    for (int i=0;i<m_body->m_numvertices;i++){
        m_body->m_vertices->at(i).xyzw[0] = (1-expansionFactor)*m_body->m_vertices->at(i).xyzw[0] + expansionFactor*orig_body->m_vertices->at(i).xyzw[0];
        m_body->m_vertices->at(i).xyzw[1] = (1-expansionFactor)*m_body->m_vertices->at(i).xyzw[1] + expansionFactor*orig_body->m_vertices->at(i).xyzw[1];
        m_body->m_vertices->at(i).xyzw[2] = (1-expansionFactor)*m_body->m_vertices->at(i).xyzw[2] + expansionFactor*orig_body->m_vertices->at(i).xyzw[2];

//        m_body->m_vertices->at(i).normal[0] = (1-expansionFactor)*m_body->m_vertices->at(i).normal[0] + expansionFactor*orig_body->m_vertices->at(i).normal[0];
//        m_body->m_vertices->at(i).normal[1] = (1-expansionFactor)*m_body->m_vertices->at(i).normal[1] + expansionFactor*orig_body->m_vertices->at(i).normal[1];
//        m_body->m_vertices->at(i).normal[2] = (1-expansionFactor)*m_body->m_vertices->at(i).normal[2] + expansionFactor*orig_body->m_vertices->at(i).normal[2];
//
//        m_body->m_vertices->at(i).uv[0] = (1-expansionFactor)*m_body->m_vertices->at(i).uv[0] + expansionFactor*orig_body->m_vertices->at(i).uv[0];
//        m_body->m_vertices->at(i).uv[1] = (1-expansionFactor)*m_body->m_vertices->at(i).uv[1] + expansionFactor*orig_body->m_vertices->at(i).uv[1];
    }

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
//    btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
//    shape->setLocalScaling(localScaling);

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
    body = createRigidBody(mass, startTransform, body_shape);

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
}

CommonExampleInterface *BasicExampleCreateFunc(CommonExampleOptions &options) {
    return new GarmentFit(options.m_guiHelper, options.m_option);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)




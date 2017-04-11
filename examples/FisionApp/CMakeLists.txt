# App_BasicExample is a minimal sample creating, stepping and deleting a Bullet dynamics world

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)

LINK_LIBRARIES(
 BulletDynamics BulletCollision LinearMath BulletSoftBody
)

IF (WIN32)
	ADD_EXECUTABLE(App_FisionExample
            GarmentFit.cpp
            main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(App_FisionExample
            GarmentFit.cpp
            main.cpp
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_FisionExample PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_FisionExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_FisionExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)





#################
# Standalone BasicExampleGui using OpenGL (but not the example browser)


INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_SOURCE_DIR}/src
        ${BULLET_PHYSICS_SOURCE_DIR}/btgui
        ${BULLET_PHYSICS_SOURCE_DIR}/examples
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
)


SET(AppFisionExample_SRCS
        GarmentFit.cpp
        ../Utils/b3ResourcePath.cpp
        ../Importers/ImportObjDemo/LoadMeshFromObj.cpp
        ../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
        ../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp

        ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	../StandaloneMain/main_opengl_single_example.cpp
        ../ExampleBrowser/OpenGLGuiHelper.cpp
        ../ExampleBrowser/GL_ShapeDrawer.cpp
        ../ExampleBrowser/CollisionShape2TriangleMesh.cpp
		../Utils/b3Clock.cpp
)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

LINK_LIBRARIES(
        BulletDynamics BulletCollision LinearMath OpenGLWindow Bullet3Common BulletSoftBody
)

#some code to support OpenGL and Glew cross platform
IF (WIN32)
        INCLUDE_DIRECTORIES(
             ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
        )
        ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
        IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

        ELSE(APPLE)
                INCLUDE_DIRECTORIES(
                        ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
                )
                ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

                LINK_LIBRARIES(  pthread dl )
        ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(AppFisionExample
                ${AppFisionExample_SRCS}
)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(AppFisionExample PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(AppFisionExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(AppFisionExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)



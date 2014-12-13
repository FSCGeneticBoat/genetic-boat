/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#include "btBulletDynamicsCommon.h"

#include "BulletHfFluid/btHfFluidRigidDynamicsWorld.h"
#include "BulletHfFluid/btHfFluid.h"
#include "BulletHfFluid/btHfFluidRigidCollisionConfiguration.h"
#include "BulletHfFluid/btHfFluidBuoyantConvexShape.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btRandom.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"

#include "HfFluidDemo.h"
#include "GL_ShapeDrawer.h"
#include "HfFluidDemo_GL_ShapeDrawer.h"

#include "GlutStuff.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 5.f;

const float TRIANGLE_SIZE=8.f;

#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_Z 1

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

#define START_POS_X 0
#define START_POS_Y 0
#define START_POS_Z 0

unsigned int current_draw_mode=DRAWMODE_NORMAL;
unsigned int current_body_draw_mode = 0;
unsigned	current_demo=0;

void Init_Drops (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid(1, 100, 100);
	btTransform xform;
	xform.setIdentity();
	xform.setOrigin(btVector3(btScalar(-50.0), btScalar(-20.0), btScalar(-50.0)));
	fluid->setWorldTransform(xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid(fluid);
	btScalar *etaArray = fluid->getEtaArray();
	
	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
		etaArray[i] = btScalar(20.0f);

	/* WAVES */
	/*for (int i = 1; i < fluid->getNumNodesWidth()-1; i++)
	{
		etaArray[fluid->arrayIndex (i, fluid->getNumNodesLength()/2-1)] = btScalar (2.0);
		etaArray[fluid->arrayIndex (i, fluid->getNumNodesLength()/2)] = btScalar (2.0);
		etaArray[fluid->arrayIndex (i, fluid->getNumNodesLength()/2+1)] = btScalar (2.0);
	}*/


	fluid->prep ();
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass = btScalar(5.0f);
		btScalar floatyness = btScalar(0.5f);
		
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
	

		float start_x = START_POS_X;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z;

		startTransform.setOrigin(btVector3(start_x,start_y,start_z));

		const btScalar boatBaseWidth = 7;
		const btScalar boatBaseLength = boatBaseWidth * 2;
		const btScalar catamaranHullsHeight = 1;
		const btScalar catamaranHullsLength = 10;
		const btScalar clothBase = boatBaseLength;
		const btScalar clothHeightOffset = 3;
		const int h = 9;
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btSoftBody* mainSail = btSoftBodyHelpers::CreatePatchUV(fluidDemo->m_softBodyWorldInfo,
			btVector3(0, clothHeightOffset, 0),
			btVector3(-clothBase, clothHeightOffset, 0),
			btVector3(0, h * 2, 0),
			btVector3(0, h * 2, 0), h, h, 0, true);


		btCompoundShape* catamaranCompound = new btCompoundShape();

		/* simple rotation from vertical to horizzontal */
		btQuaternion rot;
		rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI / 2));

		btTransform catOriginalHull1Transform, catOriginalHull1HeadTransform, catOriginalHull1TailTransform;
		btTransform catOriginalHull2Transform, catOriginalHull2HeadTransform, catOriginalHull2TailTransform;
		btTransform catTopSideTransform, catMastTreeTransform, compoundTransform;

			/* Hull 1 */
		catOriginalHull1Transform.setIdentity();
		catOriginalHull1Transform.setOrigin(btVector3(boatBaseWidth, 0, 0));
		catOriginalHull1Transform.setRotation(rot);
		
		btCylinderShape* catOriginalHull1Shape = new btCylinderShape(btVector3(catamaranHullsHeight, catamaranHullsLength, 0));
		btHfFluidBuoyantConvexShape* bytCatOriginalHull1Shape = new btHfFluidBuoyantConvexShape(catOriginalHull1Shape);
		fluidDemo->m_collisionShapes.push_back(catOriginalHull1Shape);
		fluidDemo->m_collisionShapes.push_back(bytCatOriginalHull1Shape);
		catamaranCompound->addChildShape(catOriginalHull1Transform, bytCatOriginalHull1Shape);

		catOriginalHull1HeadTransform.setIdentity();
		catOriginalHull1HeadTransform.setOrigin(btVector3(boatBaseWidth, 0, catamaranHullsLength));
		btSphereShape* catOriginalHull1HeadShape = new btSphereShape(/* radius */catamaranHullsHeight);
		btHfFluidBuoyantConvexShape* bytCatOriginalHull1HeadShape = new btHfFluidBuoyantConvexShape(catOriginalHull1HeadShape);
		fluidDemo->m_collisionShapes.push_back(catOriginalHull1HeadShape);
		fluidDemo->m_collisionShapes.push_back(bytCatOriginalHull1HeadShape);
		
		catamaranCompound->addChildShape(catOriginalHull1HeadTransform, bytCatOriginalHull1HeadShape);

		catOriginalHull1TailTransform.setIdentity();
		catOriginalHull1TailTransform.setOrigin(btVector3(boatBaseWidth, 0, -catamaranHullsLength));
		btSphereShape* catOriginalHull1TailShape = new btSphereShape(/* radius */catamaranHullsHeight);
		btHfFluidBuoyantConvexShape* bytCatOriginalHull1TailShape = new btHfFluidBuoyantConvexShape(catOriginalHull1TailShape);
		fluidDemo->m_collisionShapes.push_back(catOriginalHull1TailShape);
		fluidDemo->m_collisionShapes.push_back(bytCatOriginalHull1TailShape);
		
		catamaranCompound->addChildShape(catOriginalHull1TailTransform, bytCatOriginalHull1TailShape);
		/* Hull 2 */
		catOriginalHull2Transform.setIdentity();
		catOriginalHull2Transform.setOrigin(btVector3(-boatBaseWidth, 0, 0));
		catOriginalHull2Transform.setRotation(rot);
		
		btCylinderShape* catOriginalHull2Shape = new btCylinderShape(btVector3(catamaranHullsHeight, catamaranHullsLength, 0));
		btHfFluidBuoyantConvexShape* bytCatOriginalHull2Shape = new btHfFluidBuoyantConvexShape(catOriginalHull2Shape);
		fluidDemo->m_collisionShapes.push_back(catOriginalHull2Shape);
		fluidDemo->m_collisionShapes.push_back(bytCatOriginalHull2Shape);
		catamaranCompound->addChildShape(catOriginalHull2Transform, bytCatOriginalHull2Shape);

		catOriginalHull2HeadTransform.setIdentity();
		catOriginalHull2HeadTransform.setOrigin(btVector3(-boatBaseWidth, 0, catamaranHullsLength));
		btSphereShape* catOriginalHull2HeadShape = new btSphereShape(/* radius */catamaranHullsHeight);
		btHfFluidBuoyantConvexShape* bytCatOriginalHull2HeadShape = new btHfFluidBuoyantConvexShape(catOriginalHull2HeadShape);
		fluidDemo->m_collisionShapes.push_back(catOriginalHull2HeadShape);
		fluidDemo->m_collisionShapes.push_back(bytCatOriginalHull2HeadShape);
		
		catamaranCompound->addChildShape(catOriginalHull2HeadTransform, bytCatOriginalHull2HeadShape);

		catOriginalHull2TailTransform.setIdentity();
		catOriginalHull2TailTransform.setOrigin(btVector3(-boatBaseWidth, 0, -catamaranHullsLength));
		btSphereShape* catOriginalHull2TailShape = new btSphereShape(/* radius */catamaranHullsHeight);
		btHfFluidBuoyantConvexShape* bytCatOriginalHull2TailShape = new btHfFluidBuoyantConvexShape(catOriginalHull2TailShape);
		fluidDemo->m_collisionShapes.push_back(catOriginalHull2TailShape);
		fluidDemo->m_collisionShapes.push_back(bytCatOriginalHull2TailShape);
		
		catamaranCompound->addChildShape(catOriginalHull2TailTransform, bytCatOriginalHull2TailShape);

		/* Base */
		catTopSideTransform.setIdentity();
		catTopSideTransform.setOrigin(btVector3(0, 0.9, 0));
		btBoxShape* catBaseShape = new btBoxShape(btVector3(boatBaseWidth, 0.1, boatBaseLength/2));
		btHfFluidBuoyantConvexShape* buoyantBaseShape = new btHfFluidBuoyantConvexShape(catBaseShape);
		fluidDemo->m_collisionShapes.push_back(catBaseShape);
		fluidDemo->m_collisionShapes.push_back (buoyantBaseShape);
		
		catamaranCompound->addChildShape(catTopSideTransform, buoyantBaseShape);

		/* Mast Tree */
		catMastTreeTransform.setIdentity();
		catMastTreeTransform.setOrigin(btVector3(0, h, 0));
		btCylinderShape* catMastTreeShape = new btCylinderShape(btVector3(0.2, h, 0));
		btHfFluidBuoyantConvexShape* buoyantCatMastTreeShape = new btHfFluidBuoyantConvexShape(catMastTreeShape);
		fluidDemo->m_collisionShapes.push_back(catMastTreeShape);
		fluidDemo->m_collisionShapes.push_back (buoyantCatMastTreeShape);
		
		catamaranCompound->addChildShape(catMastTreeTransform, buoyantCatMastTreeShape);
		
		for (int i = 0; i < catamaranCompound->getNumChildShapes(); i++) {
			btCollisionShape* childShape = catamaranCompound->getChildShape(i);
			((btHfFluidBuoyantConvexShape *)(childShape))->setFloatyness(floatyness);
			((btHfFluidBuoyantConvexShape *)(childShape))->generateShape(btScalar(0.25f), btScalar(0.05f));
		}
		
		/* Compound */
		compoundTransform.setIdentity();
		compoundTransform.setOrigin(btVector3(0, 1, 0));
		if (isDynamic)
			catamaranCompound->calculateLocalInertia(mass,localInertia);
		btRigidBody::btRigidBodyConstructionInfo rbInfo = 
				btRigidBody::btRigidBodyConstructionInfo(mass,myMotionState,catamaranCompound,localInertia);
		
		btRigidBody* catamaran = new btRigidBody(rbInfo);
		fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(catamaran);

		fluidDemo->getHfFluidDynamicsWorld()->addSoftBody(mainSail);

		mainSail->getCollisionShape()->setMargin(0.5);
		btSoftBody::Material* pm = mainSail->appendMaterial();
		pm->m_kLST = 0.0004;
		pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
		mainSail->generateBendingConstraints(2, pm);

		mainSail->m_cfg.kLF = 0.05;
		mainSail->m_cfg.kDG = 0.01;

		//psb->m_cfg.kLF			=	0.004;
		//psb->m_cfg.kDG			=	0.0003;

		mainSail->m_cfg.piterations = 2;
		mainSail->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

		/*btTransform trs;
		btQuaternion rot;
		pos += btVector3(s * 2 + gap, 0, 0);
		rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI / 2));
		trs.setIdentity();
		trs.setOrigin(pos);
		trs.setRotation(rot);
		psb->transform(trs);*/
		mainSail->setTotalMass(0.3);

		mainSail->setWindVelocity(btVector3(0, 4, 40.0));
		//fluidDemo->m_cutting = true;
		fluidDemo->m_autocam = true;
		
		mainSail->appendAnchor(0, catamaran);
		mainSail->appendAnchor(h - 1, catamaran);
		mainSail->appendAnchor(h * (h - 1), catamaran);
		mainSail->appendAnchor(h * h - 1, catamaran);
		
	}
}

#define NUM_DEMOS 1

void (*demo_run_functions[NUM_DEMOS])(HfFluidDemo*)=
{
	NULL, // Run_Floatyness
};
void (*demo_init_functions[NUM_DEMOS])(HfFluidDemo*)=
{
	
	Init_Drops,
};

btScalar g_ele_array[NUM_DEMOS] = {
	btScalar(0),
	
};

btScalar g_azi_array[NUM_DEMOS] = {
	btScalar(50),
	
};

btScalar g_cameraDistance_array[NUM_DEMOS] = {
	btScalar(40),
};

#ifdef _DEBUG
const int gNumObjects = 1;
#else
const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f

//
void HfFluidDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector3 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);
		}
	}
}


void HfFluidDemo::setShootBoxShape ()
{
	if (!m_shootBoxShape)
	{
		m_shootBoxShape = new btBoxShape(btVector3(0.3f,1.f,0.2f));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape((btConvexShape*)m_shootBoxShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		m_shootBoxShape = buoyantShape;
	}
}

////////////////////////////////////

extern int gNumManifold;
extern int gOverlappingPairs;

void HfFluidDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT); 


	float dt = 1.0/60.;	

	if (m_dynamicsWorld)
	{	
		if (demo_run_functions[current_demo])
		{
			demo_run_functions[current_demo](this);
		}
	}

	if (m_dynamicsWorld)
	{
	if(m_drag)
		{
		const int				x=m_lastmousepos[0];
		const int				y=m_lastmousepos[1];
		const btVector3			rayFrom=m_cameraPosition;
		const btVector3			rayTo=getRayTo(x,y);
		const btVector3			rayDir=(rayTo-rayFrom).normalized();
		const btVector3			N=(m_cameraTargetPosition-m_cameraPosition).normalized();
		const btScalar			O=btDot(m_impact,N);
		const btScalar			den=btDot(N,rayDir);
		if((den*den)>0)
			{
			const btScalar			num=O-btDot(N,rayFrom);
			const btScalar			hit=num/den;
			if((hit>0)&&(hit<1500))
				{				
				m_goal=rayFrom+rayDir*hit;
				}				
			}		
		btVector3				delta;
		static const btScalar	maxdrag=10;
		if(delta.length2()>(maxdrag*maxdrag))
			{
			delta=delta.normalized()*maxdrag;
			}
		}
	
#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum, otherwise 4 at max
		int maxSimSubSteps = m_idle ? 1 : 4;
		//if (m_idle)
		//	dt = 1.0/420.f;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt);

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		

		//optional but useful: debug drawing
		
	}

	m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
	
#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 

	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	//gTotalContactPoints = 0;
	glutSwapBuffers();

}



void HfFluidDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	glutSwapBuffers();
}




void	HfFluidDemo::clientResetScene()
{
	DemoApplication::clientResetScene();
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(m_dynamicsWorld->getNumConstraints())
			{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
			}
		btHfFluid* hfFluid = btHfFluid::upcast(obj);
		if (hfFluid)
		{
			getHfFluidDynamicsWorld()->removeHfFluid(hfFluid);
		} else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}
	
		/* Init		*/ 

	m_softBodyWorldInfo.m_sparsesdf.Reset();

	m_softBodyWorldInfo.air_density = (btScalar) 1.2;
	m_softBodyWorldInfo.water_density = 10;
	m_softBodyWorldInfo.water_offset = 0;
	m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
	m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
	
	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
	printf("current_demo = %d\n", current_demo);
	m_azi = g_azi_array[current_demo];
	m_ele = g_ele_array[current_demo];
	m_cameraDistance = g_cameraDistance_array[current_demo];
	updateCamera();
	demo_init_functions[current_demo](this);
}

void	HfFluidDemo::renderme()
{
	btIDebugDraw* idraw = m_dynamicsWorld->getDebugDrawer();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	m_dynamicsWorld->debugDrawWorld();

	int debugMode = m_dynamicsWorld->getDebugDrawer()? m_dynamicsWorld->getDebugDrawer()->getDebugMode() : -1;

	//btHfFluidRigidDynamicsWorld* hfFluidWorld = (btHfFluidRigidDynamicsWorld*) m_dynamicsWorld;
	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	btIDebugDraw*	sdraw = softWorld->getDebugDrawer();


	for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++) {
		btSoftBody* psb = (btSoftBody*) softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe))) {
			btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
		}
	}

	/* Bodies		*/
	btVector3 ps(0, 0, 0);
	int nps = 0;

	btSoftBodyArray& sbs = softWorld->getSoftBodyArray();
	for (int ib = 0; ib < sbs.size(); ++ib) {
		btSoftBody* psb = sbs[ib];
		nps += psb->m_nodes.size();
		for (int i = 0; i < psb->m_nodes.size(); ++i) {
			ps += psb->m_nodes[i].m_x;
		}
	}
	ps /= nps;
	if (m_autocam)
		m_cameraTargetPosition += (ps - m_cameraTargetPosition)*0.05;
	/* Anm			*/
	if (!isIdle())
		m_animtime = m_clock.getTimeMilliseconds() / 1000.f;
	/* Ray cast		*/
	if (m_raycast) {
		/* Prepare rays	*/
		const int res = 64;
		const btScalar fres = res - 1;
		const btScalar size = 8;
		const btScalar dist = 10;
		btTransform trs;
		trs.setOrigin(ps);
		btScalar rayLength = 1000.f;

		const btScalar angle = m_animtime * 0.2;
		trs.setRotation(btQuaternion(angle, SIMD_PI / 4, 0));
		btVector3 dir = trs.getBasis() * btVector3(0, -1, 0);
		trs.setOrigin(ps - dir * dist);
		btAlignedObjectArray<btVector3> origins;
		btAlignedObjectArray<btScalar> fractions;
		origins.resize(res * res);
		fractions.resize(res*res, 1.f);
		for (int y = 0; y < res; ++y) {
			for (int x = 0; x < res; ++x) {
				const int idx = y * res + x;
				origins[idx] = trs * btVector3(-size + size * 2 * x / fres, dist, -size + size * 2 * y / fres);
			}
		}
		/* Cast rays	*/
		{
			m_clock.reset();
			if (sbs.size()) {
				btVector3* org = &origins[0];
				btScalar* fraction = &fractions[0];
				btSoftBody** psbs = &sbs[0];
				btSoftBody::sRayCast results;
				for (int i = 0, ni = origins.size(), nb = sbs.size(); i < ni; ++i) {
					for (int ib = 0; ib < nb; ++ib) {
						btVector3 rayFrom = *org;
						btVector3 rayTo = rayFrom + dir*rayLength;
						if (psbs[ib]->rayTest(rayFrom, rayTo, results)) {
							*fraction = results.fraction;
						}
					}
					++org;
					++fraction;
				}
				long ms = btMax<long>(m_clock.getTimeMilliseconds(), 1);
				long rayperseconds = (1000 * (origins.size() * sbs.size())) / ms;
				printf("%d ms (%d rays/s)\r\n", int(ms), int(rayperseconds));
			}
		}
		/* Draw rays	*/
		const btVector3 c[] = {origins[0],
			origins[res - 1],
			origins[res * (res - 1)],
			origins[res * (res - 1) + res - 1]};
		idraw->drawLine(c[0], c[1], btVector3(0, 0, 0));
		idraw->drawLine(c[1], c[3], btVector3(0, 0, 0));
		idraw->drawLine(c[3], c[2], btVector3(0, 0, 0));
		idraw->drawLine(c[2], c[0], btVector3(0, 0, 0));
		for (int i = 0, ni = origins.size(); i < ni; ++i) {
			const btScalar fraction = fractions[i];
			const btVector3& org = origins[i];
			if (fraction < 1.f) {
				idraw->drawLine(org, org + dir * rayLength*fraction, btVector3(1, 0, 0));
			} else {
				idraw->drawLine(org, org - dir * rayLength * 0.1, btVector3(0, 0, 0));
			}
		}
#undef RES
	}
	/* Water level	*/
	static const btVector3 axis[] = {btVector3(1, 0, 0),
		btVector3(0, 1, 0),
		btVector3(0, 0, 1)};
	if (m_softBodyWorldInfo.water_density > 0) {
		const btVector3 c = btVector3((btScalar) 0.25, (btScalar) 0.25, 1);
		const btScalar a = (btScalar) 0.5;
		const btVector3 n = m_softBodyWorldInfo.water_normal;
		const btVector3 o = -n * m_softBodyWorldInfo.water_offset;
		const btVector3 x = btCross(n, axis[n.minAxis()]).normalized();
		const btVector3 y = btCross(x, n).normalized();
		const btScalar s = 25;
		idraw->drawTriangle(o - x * s - y*s, o + x * s - y*s, o + x * s + y*s, c, a);
		idraw->drawTriangle(o - x * s - y*s, o + x * s + y*s, o - x * s + y*s, c, a);
	}
	//

	int lineWidth = 280;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	DemoApplication::renderme();	
}


void	HfFluidDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
	case	']':
		current_demo = (current_demo+1)%NUM_DEMOS;
		clientResetScene();
	break;
	case	'[':
		current_demo = (current_demo-1)%NUM_DEMOS;
		clientResetScene();
	break;
	case	'.':
		current_draw_mode = (current_draw_mode+1) % DRAWMODE_MAX;
		getHfFluidDynamicsWorld()->setDrawMode (current_draw_mode);
	break;
	case	'v':
		current_body_draw_mode = (current_body_draw_mode+1) % BODY_DRAWMODE_MAX;
		getHfFluidDynamicsWorld()->setBodyDrawMode (current_body_draw_mode);
	break;
	default:
		DemoApplication::keyboardCallback(key,x,y);
	break;
	}
}

//
void	HfFluidDemo::mouseMotionFunc(int x,int y)
{
	DemoApplication::mouseMotionFunc(x,y);
}

//
void	HfFluidDemo::mouseFunc(int button, int state, int x, int y)
{
if(button==0)
	{
	switch(state)
		{
		case	0:
			{
				DemoApplication::mouseFunc(button,state,x,y);
			}
		break;
		case	1:
				DemoApplication::mouseFunc(button,state,x,y);
		break;
		}
	}
	else
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}


void	HfFluidDemo::initPhysics()
{
///create concave ground mesh

	btCollisionShape* groundShape = 0;
	bool useConcaveMesh = false;//not ready yet true;

	if (useConcaveMesh)
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);
		
		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
	} else
	{
		groundShape = new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200));
	}

	 
	m_collisionShapes.push_back(groundShape);
	
	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	btQuaternion orn(btVector3(0,1,0),SIMD_PI);
	localTransform.setRotation(orn);
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	
	m_collisionShapes.push_back(cylinderCompound);


	m_dispatcher=0;

	/* FIXME: Register new collision algorithm */
	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btHfFluidRigidCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	
	////////////////////////////
	///Register HfFluid versus rigidbody collision algorithm


	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	m_softBodyWorldInfo.m_broadphase = m_broadphase;
	
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btHfFluidRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
	
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-22,0));



	localCreateRigidBody(0.f,tr,m_collisionShapes[0]);


	//	clientResetScene();
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	clientResetScene();
}






void	HfFluidDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}

HfFluidDemo::HfFluidDemo() : m_drag(false)
{
	overrideGLShapeDrawer (new HfFluidDemo_GL_ShapeDrawer());
	setTexturing(true);
	setShadows(true);
}

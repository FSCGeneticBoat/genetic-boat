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

#include "HfFluidDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#include <pthread.h>
#include <unistd.h>

GLDebugDrawer	gDebugDrawer;

void *windHandler(void *ptr)
{
	HfFluidDemo* fluidDemo = (HfFluidDemo *)ptr;
	bool first = true;
	sleep(2);
	fluidDemo->updateWindVelocity(0, 50, 0);
	first = false;
	while (1){
		sleep(2);
		fluidDemo->updateWindVelocity(0, 0, 0);
			
	}
}

int main(int argc,char** argv)
{
	int ret;
	pthread_t windThread;
	
	HfFluidDemo* fluidDemo = new HfFluidDemo();

	fluidDemo->initPhysics();
	
	fluidDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

	ret = pthread_create(&windThread, NULL, windHandler, (void *)fluidDemo);
	
	glutmain(argc, argv,1024,768, "Bullet Physics Demo. http://bulletphysics.com",fluidDemo);

	delete fluidDemo;
	
	return 0;
}

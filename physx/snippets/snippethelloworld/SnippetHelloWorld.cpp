//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2021 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>

#include "PxPhysicsAPI.h"

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

#include <iostream>
#include <string>

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxRigidDynamic *box, *box2;

std::string box_name = "box1";
std::string box2_name = "box2";

PxReal stackZ = 10.0f;
static int ball_index = 1;

PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
										PxFilterObjectAttributes attributes1, PxFilterData filterData1,
										PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eDETECT_DISCRETE_CONTACT
			  |	PxPairFlag::eNOTIFY_TOUCH_FOUND;

	// if (filterData0.word0 == 1 && filterData1.word0 == 1)
	// {
	// 	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT;
	// }

	// if (filterData0.word0 == 1 && filterData1.word0 == 2)
	// {
	// 	return PxFilterFlag::eKILL;
	// }
	return PxFilterFlag::eDEFAULT;
}

class MySimulationEventCallback : public PxSimulationEventCallback
{
public:
	// 碰撞开始
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override
	{
		for (PxU32 i = 0; i < nbPairs; i++)
		{
			const PxContactPair& cp = pairs[i];
            
			// 检查两个刚体是否都是动态刚体
			if (pairHeader.flags & (PxContactPairHeaderFlag::eREMOVED_ACTOR_0 | PxContactPairHeaderFlag::eREMOVED_ACTOR_1))
				continue;

			const auto actor1 = pairHeader.actors[0];
			const auto actor2 = pairHeader.actors[1];
			const auto actor1_type = actor1->getConcreteTypeName();
			const auto actor2_type = actor2->getConcreteTypeName();

			const auto event_check = cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND;
			const auto actor1_check = std::strcmp(actor1_type, "PxRigidStatic");
			const auto actor2_check = std::strcmp(actor2_type, "PxRigidStatic");
			// 碰撞信息
			if(event_check && actor1_check != 0 && actor2_check != 0)
			{
				if (pairHeader.actors[0]->getName() != nullptr && pairHeader.actors[1]->getName() != nullptr)
				{
					std::cout << "触碰开始: " << pairHeader.actors[0]->getName() << " 与 " << pairHeader.actors[1]->getName() << "\n";
				}
				// std::cout << "bingo contact" << std::endl;
			}
		}
	}

	void onTrigger(PxTriggerPair* pairs, PxU32 count)
	{
		//		printf("onTrigger: %d trigger pairs\n", count);
		while(count--)
		{
			const PxTriggerPair& current = *pairs++;
			if(current.status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
			{
				printf("Shape is entering trigger volume\n");
				auto *rigid = current.triggerActor;
				auto *in_rigid = current.otherActor;
				if (rigid->userData == nullptr || in_rigid->userData == nullptr) {
					continue;
				}
			}
			if(current.status & PxPairFlag::eNOTIFY_TOUCH_LOST)
			{
				printf("Shape is leaving trigger volume\n");
				auto *rigid = current.triggerActor;
				auto *out_rigid = current.otherActor;
				if (rigid->userData == nullptr || out_rigid->userData == nullptr) {
					continue;
				}
			}
		}
	}
	
	// 我们没有实现这些函数，但由于这是一个纯虚接口，所以我们需要提供一个空的实现
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override	{ PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count) override							{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count) override						{ PX_UNUSED(actors); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) override {}
};

MySimulationEventCallback MyCallback;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	auto *shape = gPhysics->createShape(geometry, *gMaterial);
	PxFilterData filter_data{};
	filter_data.word0 = 2;
	shape->setSimulationFilterData(filter_data);
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, *shape, 10.0f);
	const auto name = new std::string{"ball_" + std::to_string(ball_index++)};
	dynamic->setName(name->c_str());
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	PxFilterData filter_data{};
	filter_data.word0 = 1;
	shape->setSimulationFilterData(filter_data);
	for(PxU32 i = 0; i < size; i++)
	{
		for(PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			const auto name = new std::string{"box_" + std::to_string(i) + "_" + std::to_string(j)};
			body->setName(name->c_str());
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	// sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	sceneDesc.simulationEventCallback = &MyCallback;
	sceneDesc.filterShader	= contactReportFilterShader;
	sceneDesc.kineKineFilteringMode = PxPairFilteringMode::eKEEP;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.0f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	// for(PxU32 i=0;i<5;i++)
	// 	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);
	//
	// if(!interactive)
	// 	createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));

	// 测试代码
	// 在原点位置创建一个稍微大一些的长方体 trigger
	auto *trigger_shape = gPhysics->createShape(PxBoxGeometry{10, 2, 10}, *gMaterial);
	trigger_shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
	trigger_shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
	auto *actor_trigger = gPhysics->createRigidStatic(PxTransform{PxVec3{0, 3, 0}});
	actor_trigger->attachShape(*trigger_shape);
	gScene->addActor(*actor_trigger);
	trigger_shape->release();
	
	// 创建一个动态正方体用来测试
	auto *box_shape = gPhysics->createShape(PxBoxGeometry{2, 2, 2}, *gMaterial);
	box = gPhysics->createRigidDynamic(PxTransform{-15, 3, 0});
	auto filter_data = PxFilterData{1, 0, 0, 0};
	box_shape->setSimulationFilterData(filter_data);
	box->attachShape(*box_shape);
	gScene->addActor(*box);
	box_shape->release();
	box->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	box->setName(box_name.c_str());

	// // 在原点位置创建一个大长方体的静态刚体
	// auto *trigger_shape = gPhysics->createShape(PxBoxGeometry{10, 2, 10}, *gMaterial);
	// auto *actor_trigger = gPhysics->createRigidStatic(PxTransform{PxVec3{0, 3, 0}});
	// actor_trigger->attachShape(*trigger_shape);
	// gScene->addActor(*actor_trigger);
	// trigger_shape->release();
	//
	// // 在它上面创建一个动态刚体
	// auto *box_shape = gPhysics->createShape(PxBoxGeometry{2, 2, 2}, *gMaterial);
	// box = gPhysics->createRigidDynamic(PxTransform{0, 7, 0});
	// box->attachShape(*box_shape);
	// gScene->addActor(*box);
	// box_shape->release();
}

void moveBox()
{
	auto pos = box->getGlobalPose();
	pos.p.x += 1;
	box->setGlobalPose(pos);
}

void moveBoxForce() {
	box->setLinearVelocity(PxVec3{50, 0, 0}, true);
}

void delBox()
{
	gScene->removeActor(*box);
	// box->release();
}

void createBox2()
{
	if (box2 != nullptr)
	{
		gScene->removeActor(*box2);
		box2->release();
		box2 = nullptr;
	}
	// 创建一个动态正方体用来测试
	auto *box_shape = gPhysics->createShape(PxBoxGeometry{2, 2, 2}, *gMaterial);
	box2 = gPhysics->createRigidDynamic(PxTransform{-18, 3, 0});
	auto filter_data = PxFilterData{1, 0, 0, 0};
	box_shape->setSimulationFilterData(filter_data);
	box2->attachShape(*box_shape);
	gScene->addActor(*box2);
	box_shape->release();
	box2->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	box2->setName(box2_name.c_str());
}

void moveBox2()
{
	if (box2 == nullptr)
	{
		return;
	}
	auto pos = box2->getGlobalPose();
	pos.p.x += 1;
	box2->setKinematicTarget(pos);
}

void moveBoxForce2() {
	if (box2 == nullptr)
	{
		return;
	}
	box2->setLinearVelocity(PxVec3{50, 0, 0}, true);
}

void reconnectPvd()
{
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->disconnect();
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	transport->release();
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case 'B': createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
		break;
	case ' ': createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);
		break;
	case 'M': moveBox();
		break;
	case 'N': moveBoxForce();
		break;
	case 'R': delBox();
		break;
	case 'C': createBox2();
		break;
	case 'Z': moveBox2();
		break;
	case 'X': moveBoxForce2();
		break;
	case 'K': reconnectPvd();
		break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

// Box3D Solver CHOP — Phase 4: multi-node architecture.
// Owns the world settings and the time stepping of one Box3D world, which
// lives in Box3DCore.dll (shared registry keyed by this node's opId).
// Box3D Bodies CHOPs reference this node by path and contribute their own
// spawn groups; this node steps the world once per frame.
//
// This node can also spawn bodies itself (Spawn SOP or, when it is alone
// with no bodies nodes attached, a demo grid) and outputs the transforms of
// ITS OWN bodies: tx ty tz rx ry rz (degrees, TD rotate order XYZ).
// Bodies from Box3D Bodies nodes are output by those nodes, not this one.
// See touchdesigner/PLAN.md for the roadmap and the spawn attribute contract.

#pragma once

#include "CHOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"
#include "TDB3Common.h"

#include <vector>

using namespace TD;

class Box3DSolverCHOP : public CHOP_CPlusPlusBase
{
public:
	Box3DSolverCHOP( const OP_NodeInfo* info );
	virtual ~Box3DSolverCHOP();

	virtual void getGeneralInfo( CHOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual bool getOutputInfo( CHOP_OutputInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual void getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( CHOP_Output*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;
	virtual void pulsePressed( const char* name, void* reserved1 ) override;

private:
	std::vector<tdb3::SpawnBody> buildDemoGridDefs( const OP_Inputs* inputs, const tdb3::SpawnDefaults& defaults ) const;

	const uint32_t myOpId;

	// change tracking
	tdb3::SpawnDefaults myLastDefaults;
	uint32_t mySopId = 0;
	int64_t mySopCooks = -1;
	uint32_t myCollisionSopId = 0;
	int64_t myCollisionSopCooks = -1;
	int myLastBoxCount = -1;
	float myLastSpawnHeight = -1.0f;
	bool myLastDemoMode = false;
	bool myGroupRegistered = false;
	bool myResetPending = false;
};

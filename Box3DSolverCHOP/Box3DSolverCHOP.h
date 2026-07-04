// Box3D Solver CHOP — Phase 4: multi-node architecture.
// Owns the world settings and the time stepping of one Box3D world, which
// lives in Box3DCore.dll (shared registry keyed by this node's opId).
// Box3D Bodies CHOPs reference this node by path and contribute their own
// spawn groups; this node steps the world once per frame.
//
// This node does not spawn bodies or instance transforms.
// Bodies come from Box3D Body SOP / Box3D Instances CHOP only.
// The Solver output channels are kept for compatibility and stay at zero.
// See touchdesigner/PLAN.md for the roadmap and the spawn attribute contract.

#pragma once

#include "CHOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"
#include "TDB3Common.h"

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
	const uint32_t myOpId;

	// change tracking
	uint32_t myCollisionSopId = 0;
	int64_t myCollisionSopCooks = -1;
	bool myResetPending = false;
};

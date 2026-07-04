// Box3D Bodies CHOP — Phase 4.
// Contributes a group of rigid bodies to a Box3D Solver CHOP referenced by
// the Solver path parameter (no wire needed — reading the solver via
// getParCHOP creates the cook dependency, so TD steps the solver first).
// Bodies come from this node's Spawn SOP (each point = one body, same
// attribute contract as the solver, see PLAN.md) with this node's defaults.
// Outputs the transforms of ITS OWN bodies: tx ty tz rx ry rz (degrees, TD
// rotate order XYZ) — one sample per body, for per-group instancing.

#pragma once

#include "CHOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"
#include "TDB3Common.h"

#include <string>

using namespace TD;

class Box3DBodiesCHOP : public CHOP_CPlusPlusBase
{
public:
	Box3DBodiesCHOP( const OP_NodeInfo* info );
	virtual ~Box3DBodiesCHOP();

	virtual void getGeneralInfo( CHOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual bool getOutputInfo( CHOP_OutputInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual void getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( CHOP_Output*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void getWarningString( OP_String* warning, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;
	virtual void pulsePressed( const char* name, void* reserved1 ) override;

private:
	void unregisterGroup();

	const uint32_t myOpId;

	// solver binding + change tracking
	uint32_t mySolverOpId = 0;
	tdb3::SpawnDefaults myLastDefaults;
	uint32_t mySopId = 0;
	int64_t mySopCooks = -1;
	bool myGroupRegistered = false;
	bool myResetPending = false;

	std::string myWarning;
};

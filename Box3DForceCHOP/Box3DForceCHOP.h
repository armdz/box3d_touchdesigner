// Box3D Force CHOP — force fields for the simulation.
// Registers one or more force fields with a Box3D Solver CHOP (referenced by
// the Solver path parameter). The core applies them to dynamic bodies before
// every step, so bodies are pulled/pushed each frame — no wiring into the
// bodies needed.
//
// Type: Attractor (pull to a point), Repulsor (push from a point), Wind
// (constant directional force), Vortex (swirl around an axis through a point).
// Feed a Points SOP to place MANY fields at once (each point is an attractor);
// otherwise the single Position parameter is used. Target all dynamic bodies
// or restrict to one Body SOP / Instances node by path/name.
//
// Output: one sample per field with tx ty tz (world position) and strength,
// for feedback / chaining.

#pragma once

#include "CHOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"

#include <string>
#include <vector>

using namespace TD;

class Box3DForceCHOP : public CHOP_CPlusPlusBase
{
public:
	Box3DForceCHOP( const OP_NodeInfo* info );
	virtual ~Box3DForceCHOP();

	virtual void getGeneralInfo( CHOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual bool getOutputInfo( CHOP_OutputInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual void getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( CHOP_Output*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void getWarningString( OP_String* warning, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;

private:
	void unregister();

	const uint32_t myOpId;

	uint32_t mySolverOpId = 0;
	bool myRegistered = false;
	std::vector<tdb3::ForceField> myFields;

	std::string myWarning;
};

// Box3D Contacts CHOP — collision events.
// Reads the contact events captured by a Box3D Solver CHOP during its last
// advance (referenced by the Solver path parameter — reading the solver via
// getParCHOP creates the cook dependency, so TD steps the solver first).
// Output: one sample per event, empty (one zeroed sample, active=0) when
// nothing happened this frame. Events live exactly one frame.
//
// Channels: active, kind (0 begin / 1 end / 2 hit), idxa/idxb (body index
// inside its group), worlda/worldb (1 = that side is the ground / container
// walls / Collision SOP mesh), px py pz (world contact point), nx ny nz
// (contact normal A->B), speed (hit approach speed, m/s).
// The Info DAT lists the same events with the node paths of both bodies.
//
// The optional Body filter keeps only events involving one Body SOP /
// Instances CHOP (path or node name) and normalizes them so that body is
// always side A — idxa then indexes straight into that node's instances.

#pragma once

#include "CHOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"
#include "TDB3Common.h"

#include <string>
#include <vector>

using namespace TD;

class Box3DContactsCHOP : public CHOP_CPlusPlusBase
{
public:
	Box3DContactsCHOP( const OP_NodeInfo* info );
	virtual ~Box3DContactsCHOP();

	virtual void getGeneralInfo( CHOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual bool getOutputInfo( CHOP_OutputInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual void getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( CHOP_Output*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual bool getInfoDATSize( OP_InfoDATSize* infoSize, void* reserved1 ) override;
	virtual void getInfoDATEntries( int32_t index, int32_t nEntries, OP_InfoDATEntries* entries,
									void* reserved1 ) override;

	virtual void getWarningString( OP_String* warning, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;

private:
	// Pulls this frame's events from the solver, applying the kind toggles and
	// the body filter. Called from getOutputInfo (which sizes the output);
	// execute reuses the cached list.
	void fetchEvents( const OP_Inputs* inputs );

	const uint32_t myOpId;

	uint32_t mySolverOpId = 0;
	std::vector<tdb3::ContactEvent> myEvents;
	int myBeginCount = 0;
	int myEndCount = 0;
	int myHitCount = 0;

	std::string myWarning;
};

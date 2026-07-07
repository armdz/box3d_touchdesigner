#pragma once

#include "CHOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"

#include <string>
#include <vector>

using namespace TD;

class Box3DJointCHOP : public CHOP_CPlusPlusBase
{
public:
	Box3DJointCHOP( const OP_NodeInfo* info );
	virtual ~Box3DJointCHOP();

	virtual void getGeneralInfo( CHOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual bool getOutputInfo( CHOP_OutputInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual void getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( CHOP_Output*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void getWarningString( OP_String* warning, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;

private:
	void unregisterJoint();

	const uint32_t myOpId;
	uint32_t mySolverOpId = 0;
	bool myJointRegistered = false;

	// Live anchors + resolve state, one entry per registered joint (output
	// sample). Order matches the specs list: pair-major, series index minor.
	struct JointState
	{
		float a[3] = { 0.0f, 0.0f, 0.0f };
		float b[3] = { 0.0f, 0.0f, 0.0f };
		bool active = false;
	};
	std::vector<JointState> myStates;
	bool myActive = false; // any joint resolved
	std::string myWarning;
};

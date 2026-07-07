#pragma once

#include "SOP_CPlusPlusBase.h"

#include <string>

using namespace TD;

class Box3DSetJointSOP : public SOP_CPlusPlusBase
{
public:
	Box3DSetJointSOP( const OP_NodeInfo* info );
	virtual ~Box3DSetJointSOP();

	virtual void getGeneralInfo( SOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;
	virtual void execute( SOP_Output*, const OP_Inputs*, void* reserved1 ) override;
	virtual void executeVBO( SOP_VBOOutput*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;

private:
	float myPivot[3] = { 0.0f, 0.0f, 0.0f };
	bool myEnabled = false;
};

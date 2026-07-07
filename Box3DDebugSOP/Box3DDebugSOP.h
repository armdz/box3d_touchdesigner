// Box3D Debug SOP — debug draw of the live collision world (Bullet-style).
// References a Box3D Solver CHOP by path and outputs the REAL collision
// shapes box3d is using (hulls after the vertex-budget simplification, meshes
// after welding) as world-space wireframe line primitives, colored by state:
// dynamic awake green, dynamic asleep blue, kinematic orange, static gray,
// ground/walls/collision mesh dark gray, joints yellow.
//
// Intended usage: standalone node with the Display flag ON and the Render
// flag OFF — visible in the viewport, absent from Render TOPs. Or feed it to
// a dedicated Geometry COMP with a wireframe/line MAT.

#pragma once

#include "SOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"

#include <string>
#include <vector>

using namespace TD;

class Box3DDebugSOP : public SOP_CPlusPlusBase
{
public:
	Box3DDebugSOP( const OP_NodeInfo* info );
	virtual ~Box3DDebugSOP();

	virtual void getGeneralInfo( SOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( SOP_Output*, const OP_Inputs*, void* reserved1 ) override;
	virtual void executeVBO( SOP_VBOOutput*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void getWarningString( OP_String* warning, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;

private:
	int mySegmentCount = 0;
	std::string myWarning;
};

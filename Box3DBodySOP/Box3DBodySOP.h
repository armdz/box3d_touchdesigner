// Box3D Body SOP — Phase 4: a single rigid body as a SOP.
// Wire your geometry in (optional) and reference a Box3D Solver CHOP by
// path; this node registers ONE body in that world and outputs its geometry
// transformed by the simulation every frame — no instancing needed.
//
// Collision shape (Shape parameter):
//   Input Hull — convex hull of the input SOP points (body origin = centroid)
//   Box / Sphere / Capsule — primitive of Size, at the input centroid, or at
//   the Position parameter when no input is wired (a matching preview mesh
//   is generated as output in that case).
//
// The body transform is also exposed on the Info CHOP (tx ty tz rx ry rz).
// See touchdesigner/PLAN.md for the roadmap.

#pragma once

#include "SOP_CPlusPlusBase.h"

#include "Box3DTDCore.h"

#include <string>
#include <vector>

using namespace TD;

class Box3DBodySOP : public SOP_CPlusPlusBase
{
public:
	Box3DBodySOP( const OP_NodeInfo* info );
	virtual ~Box3DBodySOP();

	virtual void getGeneralInfo( SOP_GeneralInfo*, const OP_Inputs*, void* reserved1 ) override;

	virtual void execute( SOP_Output*, const OP_Inputs*, void* reserved1 ) override;
	virtual void executeVBO( SOP_VBOOutput*, const OP_Inputs*, void* reserved1 ) override;

	virtual int32_t getNumInfoCHOPChans( void* reserved1 ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* reserved1 ) override;

	virtual void getWarningString( OP_String* warning, void* reserved1 ) override;

	virtual void setupParameters( OP_ParameterManager* manager, void* reserved1 ) override;
	virtual void pulsePressed( const char* name, void* reserved1 ) override;

private:
	struct BodySettings
	{
		int shape = 0; // menu: 0 input hull, 1 box, 2 sphere, 3 capsule
		float sizeX = 1.0f, sizeY = 1.0f, sizeZ = 1.0f;
		float posX = 0.0f, posY = 3.0f, posZ = 0.0f;
		int type = 0; // menu: 0 dynamic, 1 kinematic, 2 static
		float density = 1.0f;
		float friction = 0.6f;
		float restitution = 0.0f;

		bool operator!=( const BodySettings& o ) const
		{
			return shape != o.shape || sizeX != o.sizeX || sizeY != o.sizeY || sizeZ != o.sizeZ || posX != o.posX ||
				   posY != o.posY || posZ != o.posZ || type != o.type || density != o.density ||
				   friction != o.friction || restitution != o.restitution;
		}
	};

	BodySettings readSettings( const OP_Inputs* inputs ) const;
	void unregisterGroup();
	void outputTransformedInput( SOP_Output* output, const OP_SOPInput* input, const tdb3::BodyTransform& t );
	void outputPrimitiveMesh( SOP_Output* output, const BodySettings& s, const tdb3::BodyTransform& t );

	const uint32_t myOpId;

	// solver binding + change tracking
	uint32_t mySolverOpId = 0;
	BodySettings myLastSettings;
	uint32_t mySopId = 0;
	int64_t mySopCooks = -1;
	bool myGroupRegistered = false;
	bool myResetPending = false;

	// body origin used at spawn (input centroid or Position parameter);
	// output geometry is rebuilt relative to this every frame
	float myCentroid[3] = { 0.0f, 0.0f, 0.0f };

	// Cached frame for input-driven rigid animation. For Input Hull we keep the
	// local hull points stable and update pose (p,q) from this frame, so
	// Transform SOP animation does not force a full-world rebuild every frame.
	bool myInputFrameValid = false;
	// Source rigid frame quaternion from the current input SOP cook. Used to
	// rotate normals with a relative transform when local hull caching is active.
	float myInputSourceQ[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	int myInputPointCount = 0;
	int myInputAnchor[3] = { 0, 0, 0 };
	float myInputRefEdgeLen[3] = { 0.0f, 0.0f, 0.0f };
	std::vector<float> myHullLocalPoints;

	// last transform, for the Info CHOP
	tdb3::BodyTransform myTransform = { 0, 0, 0, 0, 0, 0, 1 };

	std::string myWarning;
};

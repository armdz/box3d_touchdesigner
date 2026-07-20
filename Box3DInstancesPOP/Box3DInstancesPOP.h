// Box3D Instances POP — GPU/POP rigid-body instancing for live performance.
// A POP counterpart of the Box3D Instances CHOP: the input POP's points are the
// spawn points (one rigid body per point, count variable live), and the output
// is an N-point cloud carrying each body's live pose for instancing geometry on
// the GPU with no channel round-trip. Shares the Solver (same box3d world) with
// every other body group, so they collide for free.
//
// Output attributes (Point class): P (float3 position), rot (float3 rotation as
// TouchDesigner Euler XYZ degrees), scale (float3), v (float3 linear velocity).
//
// Per-point input attributes (all optional, override the node defaults): scale
// or size (float3/float1), shape (int 0 box / 1 sphere / 2 capsule), density,
// friction, restitution, type (int 0 static / 1 kinematic / 2 dynamic), alive
// (0 static / 1 dynamic — overrides type), bullet (CCD), orient (float4 quat) or
// rot (float3 Euler XYZ degrees).

#pragma once

#include "POP_CPlusPlusBase.h"

#include "Box3DTDCore.h"

#include <string>
#include <vector>

using namespace TD;

class Box3DInstancesPOP : public POP_CPlusPlusBase
{
public:
	Box3DInstancesPOP( const OP_NodeInfo* info, POP_Context* context );
	virtual ~Box3DInstancesPOP();

	virtual void getGeneralInfo( POP_GeneralInfo*, const OP_Inputs*, void* ) override;
	virtual void execute( POP_Output*, const OP_Inputs*, void* ) override;
	virtual int32_t getNumInfoCHOPChans( void* ) override;
	virtual void getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* ) override;
	virtual void getWarningString( OP_String*, void* ) override;
	virtual void setupParameters( OP_ParameterManager*, void* ) override;
	virtual void pulsePressed( const char*, void* ) override;

private:
	// Per-cook body defaults read from the parameters.
	struct Defaults
	{
		int shape = 0;
		float sizeX = 1.0f, sizeY = 1.0f, sizeZ = 1.0f;
		float density = 1.0f;
		float friction = 0.6f;
		float restitution = 0.0f;
		int type = 2;
		bool bullet = false;

		bool operator!=( const Defaults& o ) const
		{
			return shape != o.shape || sizeX != o.sizeX || sizeY != o.sizeY || sizeZ != o.sizeZ ||
				   density != o.density || friction != o.friction || restitution != o.restitution ||
				   type != o.type || bullet != o.bullet;
		}
	};

	void unregisterGroup();
	Defaults readDefaults( const OP_Inputs* inputs ) const;
	// Turns the input POP's points into spawn bodies (one per point).
	std::vector<tdb3::SpawnBody> parseSpawnPOP( const OP_POPInput* in, const Defaults& defaults ) const;

	const OP_NodeInfo* myNodeInfo;
	POP_Context* myContext;
	const uint32_t myOpId;

	uint32_t mySolverOpId = 0;
	bool myRegistered = false;
	bool myResetPending = false;

	// change tracking so we only rebuild the group when something changed
	bool myHaveLast = false;
	Defaults myLastDefaults;
	uint32_t myPopId = 0;
	int64_t myPopCooks = -1;
	std::vector<tdb3::SpawnBody> myLastSpawnDefs;

	std::string myWarning;
};

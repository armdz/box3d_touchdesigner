// Shared core for the Box3D TouchDesigner plugins.
//
// One custom operator per DLL is a TD constraint, so the Solver and Bodies
// operators live in separate plugin DLLs. They share simulation state through
// this core DLL: a registry of SolverCore instances keyed by the solver
// node's opId (unique per TD process). Box3D itself is statically linked
// INTO this DLL and is not exported — plugin DLLs must go through this API
// (box3d inline header math is fine).
//
// Threading: TD cooks C++ operators on the cook thread one at a time, so
// this core is intentionally lock-free. Do not call it from other threads.

#pragma once

#include <cstdint>
#include <vector>

#if defined( BOX3DTD_BUILD )
#define BOX3DTD_API __declspec( dllexport )
#else
#define BOX3DTD_API __declspec( dllimport )
#endif

namespace tdb3
{

/// Everything needed to (re)create one rigid body.
struct SpawnBody
{
	float px = 0.0f, py = 0.0f, pz = 0.0f;
	float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
	int shape = 0; // 0 box, 1 sphere, 2 capsule, 3 convex hull
	float sizeX = 0.5f, sizeY = 0.5f, sizeZ = 0.5f; // full sizes (primitive shapes)
	float density = 1.0f;
	float friction = 0.6f;
	float restitution = 0.0f;
	int type = 2; // 0 static, 1 kinematic, 2 dynamic

	// Convex hull source points (xyz triplets, relative to the body origin).
	// Used when shape == 3; box3d computes the hull with a vertex budget.
	// Falls back to a box when the hull cannot be built.
	std::vector<float> hullPoints;
};

struct BodyTransform
{
	float px, py, pz;
	float qx, qy, qz, qw;
};

struct WorldSettings
{
	float gravityX = 0.0f, gravityY = -10.0f, gravityZ = 0.0f;
	bool ground = true;
	float groundSize = 20.0f;
	int subSteps = 4;
};

class BOX3DTD_API SolverCore
{
public:
	~SolverCore();

	// Gravity applies live; ground changes mark the world for rebuild.
	void setWorldSettings( const WorldSettings& settings );

	// Static collision mesh as a raw triangle soup (xyz floats, 3 indices per
	// triangle). Copied. Marks the world for rebuild.
	void setCollisionMesh( const float* vertices, int vertexCount, const int32_t* indices, int indexCount );
	void clearCollisionMesh();

	// Register/update/remove a spawn group. Any change rebuilds the whole
	// world on the next advance(), respawning every group (deterministic).
	void setGroup( uint32_t groupKey, std::vector<SpawnBody> defs );
	void removeGroup( uint32_t groupKey );
	bool hasGroup( uint32_t groupKey ) const;
	int groupCount() const;

	void requestRebuild();

	// Rebuild if needed, then run fixed 60 Hz steps out of dtSeconds.
	void advance( double dtSeconds, bool simulate );

	int getGroupBodyCount( uint32_t groupKey ) const;

	// Fills out[0..capacity). Falls back to the spawn definitions when the
	// group has not been built yet (new group waiting for the next rebuild).
	// Returns the number of transforms written.
	int getGroupTransforms( uint32_t groupKey, BodyTransform* out, int capacity ) const;

	int totalBodyCount() const;
	int awakeBodyCount() const;
	int64_t stepCount() const;

private:
	friend class Registry;
	SolverCore();
	SolverCore( const SolverCore& ) = delete;
	SolverCore& operator=( const SolverCore& ) = delete;

	struct Impl;
	Impl* myImpl;
};

/// Process-wide map of solver nodes to their cores, keyed by the solver
/// node's opId. Lives in this DLL so every plugin DLL sees the same map.
class BOX3DTD_API Registry
{
public:
	static SolverCore* getOrCreate( uint32_t solverOpId );
	static SolverCore* find( uint32_t solverOpId );
	static void destroy( uint32_t solverOpId );
};

} // namespace tdb3

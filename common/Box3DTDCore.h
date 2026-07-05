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
#include <string>
#include <vector>

#if defined( _WIN32 )
	#if defined( BOX3DTD_BUILD )
	#define BOX3DTD_API __declspec( dllexport )
	#else
	#define BOX3DTD_API __declspec( dllimport )
	#endif
#else
#define BOX3DTD_API __attribute__( ( visibility( "default" ) ) )
#endif

namespace tdb3
{

/// Everything needed to (re)create one rigid body.
struct SpawnBody
{
	float px = 0.0f, py = 0.0f, pz = 0.0f;
	float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
	int shape = 0; // 0 box, 1 sphere, 2 capsule, 3 convex hull, 4 triangle mesh (concave OK; static/kinematic only)
	float sizeX = 1.0f, sizeY = 1.0f, sizeZ = 1.0f; // full sizes (primitive shapes)
	float density = 1.0f;
	float friction = 0.6f;
	float restitution = 0.0f;
	int type = 2; // 0 static, 1 kinematic, 2 dynamic
	// Continuous collision (bullet) for fast dynamics: prevents tunneling
	// through kinematic/dynamic geometry (vs static it is always on).
	bool bullet = false;
	bool jointEnabled = false;
	float jointPivotX = 0.0f, jointPivotY = 0.0f, jointPivotZ = 0.0f;

	// Convex hull source points (xyz triplets, relative to the body origin).
	// Used when shape == 3; box3d computes the hull with a vertex budget.
	// Falls back to a box when the hull cannot be built.
	// For shape == 4 these are the exact mesh vertices instead.
	std::vector<float> hullPoints;

	// Triangle indices into hullPoints (3 per triangle), shape == 4 only.
	// The mesh is used as-is (no convex approximation), so concave geometry
	// like terrain collides per-triangle. Dynamic bodies cannot use meshes
	// (engine limit); the core falls back to a convex hull for them.
	std::vector<int32_t> meshIndices;
};

struct BodyTransform
{
	float px, py, pz;
	float qx, qy, qz, qw;
};

/// Full per-body simulation state, for optional CHOP channels.
struct BodyState
{
	float px, py, pz;
	float qx, qy, qz, qw;
	float vx, vy, vz; // linear velocity
	float wx, wy, wz; // angular velocity (radians/second)
	float awake;	  // 1 awake, 0 asleep
};

/// One joint between two bodies, declared by the Solver's Joints DAT.
/// Bodies are referenced by the registered path of their owner node (Body
/// SOP / Instances CHOP) plus an index into that group. An empty bodyB means
/// the joint anchors body A to the static world.
/// Resolution is lazy: rows that do not match a registered group yet are
/// retried whenever groups or paths change.
struct JointSpec
{
	int type = 0; // 0 distance, 1 spherical, 2 revolute, 3 weld
	std::string bodyA;
	int indexA = 0;
	std::string bodyB;
	int indexB = 0;

	// Where the joint pivots: 0 body A origin, 1 body B origin, 2 the explicit
	// anchor point below. Distance joints attach at each body origin regardless
	// (the pivot is only the world-anchor attach point).
	int pivotMode = 0;
	float anchorX = 0.0f, anchorY = 0.0f, anchorZ = 0.0f;

	// Joint frame z-axis in world space: revolute hinge axis / spherical cone
	// axis. Defaults to +Z.
	float axisX = 0.0f, axisY = 0.0f, axisZ = 1.0f;

	// Spring (distance/spherical/revolute: hertz > 0 enables; weld: stiffness)
	float hertz = 0.0f;
	float dampingRatio = 0.0f;

	// Distance joint. length < 0 = use the distance at creation time.
	float length = -1.0f;
	bool enableLimit = false; // distance: min/max length; revolute: lower/upper angle
	float minLength = 0.0f;
	float maxLength = 0.0f;

	// Revolute limits / spherical twist limits (radians).
	float lowerAngle = 0.0f;
	float upperAngle = 0.0f;

	// Spherical cone limit (radians).
	bool enableConeLimit = false;
	float coneAngle = 0.0f;

	// Motor (revolute: rad/s + N*m; distance: m/s + N).
	bool enableMotor = false;
	float motorSpeed = 0.0f;
	float maxMotorForce = 0.0f;

	bool collideConnected = false;
};

struct WorldSettings
{
	float gravityX = 0.0f, gravityY = -10.0f, gravityZ = 0.0f;
	float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
	bool ground = true;
	float groundSize = 20.0f;
	bool container = false;
	float wallHeight = 4.0f;
	float wallThickness = 0.25f;
	int workerCount = 1;
	int subSteps = 4;
	int maxStepsPerCook = 8;
	// Let quiet bodies fall asleep. Off = every body simulates every step.
	bool sleep = true;
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

	// Register/update/remove a spawn group. While the world is live, updates
	// apply to just that group: material changes are set on the existing
	// shapes; pose changes move static/kinematic bodies (dynamic bodies keep
	// simulating and only take spawn poses at creation or on a rebuild);
	// shape/size/type/count changes recreate the group's bodies, preserving
	// the simulated pose and velocity of dynamic bodies whose spawn pose is
	// unchanged. When the world is pending a rebuild, the group respawns with
	// everything else on the next advance() (deterministic).
	void setGroup( uint32_t groupKey, std::vector<SpawnBody> defs );
	void removeGroup( uint32_t groupKey );
	bool hasGroup( uint32_t groupKey ) const;
	int groupCount() const;

	// Registers the TD path of the node owning a group, so Joints DAT rows
	// can reference bodies by node path (or bare node name).
	void setGroupPath( uint32_t groupKey, const char* path );

	int activeJointCount() const;

	// Register/update/remove one or more joints owned by a node (SOP/CHOP),
	// keyed by that node's opId. Same lifetime rules as other joints.
	void setJointNodeList( uint32_t ownerKey, const std::vector<JointSpec>& specs );
	void removeJointNode( uint32_t ownerKey );

	// Live world-space anchor points of one owned node joint (for drawing/state).
	// Returns false while the joint is unresolved or the world is not built.
	bool getJointAnchors( uint32_t ownerKey, int jointIndex, float outA[3], float outB[3] ) const;

	void requestRebuild();

	// Rebuild if needed, then run fixed 60 Hz steps out of dtSeconds.
	void advance( double dtSeconds, bool simulate );

	int getGroupBodyCount( uint32_t groupKey ) const;

	// Fills out[0..capacity). Falls back to the spawn definitions when the
	// group has not been built yet (new group waiting for the next rebuild).
	// Returns the number of transforms written.
	int getGroupTransforms( uint32_t groupKey, BodyTransform* out, int capacity ) const;

	// Same as getGroupTransforms but with velocities and awake state (zeros
	// when falling back to spawn definitions).
	int getGroupStates( uint32_t groupKey, BodyState* out, int capacity ) const;

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

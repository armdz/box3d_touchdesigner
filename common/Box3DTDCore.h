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
	// 0 box, 1 sphere, 2 capsule, 3 convex hull, 4 triangle mesh (concave OK;
	// static/kinematic only), 5 compound of convex hulls (concave OK, dynamic OK),
	// 6 hollow box / container (thin wall slabs so collision faces INWARD —
	// objects stay inside; dynamic OK)
	int shape = 0;
	float sizeX = 1.0f, sizeY = 1.0f, sizeZ = 1.0f; // full sizes (primitive shapes)

	// Hollow box (shape 6): wall thickness and whether the +Y face is left open
	// (drop things in). Ignored by other shapes.
	float wallThickness = 0.1f;
	bool openTop = false;
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

	// shape == 5 only: hullPoints is the concatenation of several pieces and
	// this holds the point count of each piece. Each piece becomes one convex
	// hull shape on the same body (industry-standard convex composition), so
	// concave objects modeled in pieces can be fully dynamic.
	std::vector<int32_t> hullPieceCounts;
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

/// One collision event captured during the last advance() (all fixed steps of
/// that cook are merged into one buffer, cleared at the next simulating
/// advance). Bodies are identified by owner group key (the client node's
/// opId) + body index inside that group; group 0 means "the world" (ground,
/// container walls, the Solver's Collision SOP mesh, or any static outside a
/// group).
struct ContactEvent
{
	int kind = 0; // 0 begin touch, 1 end touch, 2 hit (impact above the world hit-speed threshold)
	uint32_t groupA = 0;
	int indexA = -1;
	uint32_t groupB = 0;
	int indexB = -1;
	// World-space contact point and normal (normal points from A to B).
	// Always filled for hits; filled for begin events when the contact
	// manifold is available; zero for end events.
	float px = 0.0f, py = 0.0f, pz = 0.0f;
	float nx = 0.0f, ny = 0.0f, nz = 0.0f;
	// Approach speed of the impact (hit events only, always positive).
	float speed = 0.0f;
};

/// Live per-body contact info, for optional CHOP channels.
struct BodyContactState
{
	float touching = 0.0f; // number of touching contacts right now
	float impulse = 0.0f;  // sum of total normal impulses over touching manifolds (N*s)
	float hitSpeed = 0.0f; // strongest hit approach speed captured in the last advance, 0 if none
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

	// Where the joint anchors:
	// 0 = each body's own pivot (pivot-to-pivot, Bullet style: the solver
	//     pulls both pivots together until they coincide),
	// 1 = body A's pivot anchors BOTH bodies,
	// 2 = body B's pivot anchors BOTH bodies (child-bone convention: the child
	//     bone carries the anchor, the parent's own pivot is not consulted —
	//     this is what lets one pivot per body articulate a whole skeleton),
	// 3 = the explicit world-space anchor point below.
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

/// A live grab (mouse-joint style): a zero-length spring constraint between a
/// body-local point of one group body and a moving world target. The core
/// drives the target through a hidden shapeless KINEMATIC body retargeted with
/// b3Body_SetTargetTransform every step, so dragging imparts real velocity and
/// releasing keeps the momentum. Registered per owner node like forces; the
/// target/spring update live (no joint recreation) as long as the grabbed
/// body-point stays the same.
struct GrabSpec
{
	uint32_t groupKey = 0; // group that owns the grabbed body
	int bodyIndex = 0;	   // body inside that group
	// grabbed point, in the body's local frame (latched at grab time)
	float localX = 0.0f, localY = 0.0f, localZ = 0.0f;
	// world-space target the grabbed point is pulled toward (moves every cook)
	float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f;
	float hertz = 15.0f;	   // spring strength; 0 = rigid hold
	float dampingRatio = 1.0f; // spring damping
};

/// A force field applied to bodies before each simulation step (attractor,
/// repulsor, directional wind, or vortex). Registered per owner node like
/// joints; the core applies it every step until the node stops cooking.
struct ForceField
{
	int type = 0; // 0 attractor (pull to point), 1 repulsor (push from point), 2 directional (wind), 3 vortex
	float px = 0.0f, py = 0.0f, pz = 0.0f; // point (attractor/repulsor/vortex center)
	float dx = 0.0f, dy = 1.0f, dz = 0.0f; // direction (wind) / axis (vortex)
	float strength = 10.0f;
	float radius = 0.0f; // influence radius; 0 = unlimited
	int falloff = 0;	 // 0 none, 1 linear (to radius), 2 inverse-square
	bool useMass = false; // false = same acceleration for all bodies (F = m*a), true = same force
	// 0 = every dynamic body in the world; otherwise only the group with this
	// key (a Body SOP / Instances node's opId).
	uint32_t targetGroup = 0;
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
	// Collision speed (m/s) needed to generate a hit event. Applies live.
	float hitThreshold = 1.0f;
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

	// Register/update/remove the force fields owned by a node (keyed by opId).
	// Applied to bodies before every step; removed when the node stops cooking
	// (same heartbeat rule as groups/joints, so bypassing turns the force off).
	void setForceNodeList( uint32_t ownerKey, const std::vector<ForceField>& fields );
	void removeForceNode( uint32_t ownerKey );

	// Register/update/remove the live grabs owned by a node (keyed by opId).
	// Call every cook with the current list: matching grabs (same group/body/
	// local point) keep their live constraint and only move the target /
	// retune the spring; changed or removed entries recreate or release.
	// Same heartbeat rule as forces — a node that stops cooking drops its
	// grabs (with their momentum) within a few frames.
	void setGrabList( uint32_t ownerKey, const std::vector<GrabSpec>& grabs );
	void removeGrabNode( uint32_t ownerKey );

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

	// Collision events captured during the last simulating advance() (empty
	// while paused or before the world is built). Fixed-size read: query the
	// count first, then fill.
	int contactEventCount() const;
	int getContactEvents( ContactEvent* out, int capacity ) const;

	// Live per-body contact state of one group: touching contact count, summed
	// normal impulse, and the strongest hit speed seen in the last advance.
	// Zeros when the group is not built yet. Returns the number written.
	int getGroupContactStates( uint32_t groupKey, BodyContactState* out, int capacity ) const;

	// Registered TD path of a group (empty string when unknown). For the
	// Contacts CHOP Info DAT, which reports events by node path.
	bool getGroupPathByKey( uint32_t groupKey, std::string& outPath ) const;

	// Resolve a Body SOP / Instances CHOP reference (full path or bare node
	// name) to its group key. Used by the Contacts CHOP body filter.
	bool findGroupKeyByPath( const char* ref, uint32_t& outKey ) const;

	int totalBodyCount() const;
	int awakeBodyCount() const;
	int64_t stepCount() const;

	// Debug wireframe of the live collision world, read back from box3d itself
	// (post hull simplification / post weld), in world space at the live body
	// poses. Appends line segments (6 floats: x0 y0 z0 x1 y1 z1) plus one RGB
	// color (3 floats) per segment. Colors: dynamic awake green, dynamic
	// asleep blue, kinematic orange, static gray, world statics (ground/walls/
	// collision mesh) dark gray, joints yellow. Empty until the world is built.
	void getDebugWireframe( bool bodies, bool worldStatics, bool joints, std::vector<float>& segments,
							std::vector<float>& colors ) const;

	// Same wireframe but for one group's bodies only — used by the body
	// nodes' Show Collision Shape toggle to draw their own collider inline.
	void getGroupWireframe( uint32_t groupKey, std::vector<float>& segments, std::vector<float>& colors ) const;

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

#include "Box3DTDCore.h"

#include "box3d/box3d.h"

#include <cmath>
#include <map>

namespace tdb3
{

namespace
{

constexpr double kFixedTimeStep = 1.0 / 60.0;
constexpr int kHullVertexBudget = 32;

struct Group
{
	std::vector<SpawnBody> defs;
	std::vector<b3BodyId> bodies;
	std::string path; // TD path of the owner node, for Joints DAT references

	// Per-group mesh blobs for shape == 4 bodies. box3d references (does not
	// copy) b3MeshData, so these must outlive the bodies/shapes using them
	// and be destroyed together with them.
	std::vector<b3MeshData*> meshes;
};

bool sameJointSpec( const JointSpec& a, const JointSpec& b )
{
	return a.type == b.type && a.bodyA == b.bodyA && a.indexA == b.indexA && a.bodyB == b.bodyB &&
		   a.indexB == b.indexB && a.pivotMode == b.pivotMode && a.anchorX == b.anchorX && a.anchorY == b.anchorY &&
		   a.anchorZ == b.anchorZ && a.axisX == b.axisX && a.axisY == b.axisY && a.axisZ == b.axisZ &&
		   a.hertz == b.hertz && a.dampingRatio == b.dampingRatio && a.length == b.length &&
		   a.enableLimit == b.enableLimit && a.minLength == b.minLength && a.maxLength == b.maxLength &&
		   a.lowerAngle == b.lowerAngle && a.upperAngle == b.upperAngle && a.enableConeLimit == b.enableConeLimit &&
		   a.coneAngle == b.coneAngle && a.enableMotor == b.enableMotor && a.motorSpeed == b.motorSpeed &&
		   a.maxMotorForce == b.maxMotorForce && a.collideConnected == b.collideConnected;
}

// Does a Joints DAT cell reference this group? Accepts the full TD path or
// just the node name (last path component).
bool pathMatches( const std::string& groupPath, const std::string& ref )
{
	if ( groupPath.empty() || ref.empty() )
	{
		return false;
	}
	if ( groupPath == ref )
	{
		return true;
	}
	size_t slash = groupPath.find_last_of( '/' );
	return slash != std::string::npos && groupPath.compare( slash + 1, std::string::npos, ref ) == 0;
}

// Quaternion rotating +Z onto the given (non-normalized) axis; identity when
// the axis is degenerate.
b3Quat quatFromZAxis( float ax, float ay, float az )
{
	float lengthSq = ax * ax + ay * ay + az * az;
	if ( lengthSq < 1e-12f )
	{
		return b3Quat_identity;
	}

	float inv = 1.0f / sqrtf( lengthSq );
	float x = ax * inv, y = ay * inv, z = az * inv;

	// axis ≈ -Z: rotate 180 degrees around X
	if ( z < -0.999999f )
	{
		b3Quat q;
		q.v.x = 1.0f;
		q.v.y = 0.0f;
		q.v.z = 0.0f;
		q.s = 0.0f;
		return q;
	}

	// shortest arc from (0,0,1): q = (cross(z, axis), 1 + dot) normalized
	b3Quat q;
	q.v.x = -y;
	q.v.y = x;
	q.v.z = 0.0f;
	q.s = 1.0f + z;
	return b3NormalizeQuat( q );
}

b3Transform toLocalTransform( b3BodyId bodyId )
{
	b3WorldTransform wt = b3Body_GetTransform( bodyId );
	b3Transform t;
	t.p = b3Vec3{ (float)wt.p.x, (float)wt.p.y, (float)wt.p.z };
	t.q = wt.q;
	return t;
}

bool sameShapeAndType( const SpawnBody& a, const SpawnBody& b )
{
	if ( a.shape != b.shape || a.sizeX != b.sizeX || a.sizeY != b.sizeY || a.sizeZ != b.sizeZ || a.type != b.type ||
		 a.bullet != b.bullet )
	{
		return false;
	}

	if ( a.hullPoints.size() != b.hullPoints.size() )
	{
		return false;
	}

	for ( size_t i = 0; i < a.hullPoints.size(); ++i )
	{
		if ( a.hullPoints[i] != b.hullPoints[i] )
		{
			return false;
		}
	}

	if ( a.meshIndices.size() != b.meshIndices.size() )
	{
		return false;
	}

	for ( size_t i = 0; i < a.meshIndices.size(); ++i )
	{
		if ( a.meshIndices[i] != b.meshIndices[i] )
		{
			return false;
		}
	}

	return true;
}

bool sameMaterial( const SpawnBody& a, const SpawnBody& b )
{
	return a.density == b.density && a.friction == b.friction && a.restitution == b.restitution;
}

bool samePose( const SpawnBody& a, const SpawnBody& b )
{
	return a.px == b.px && a.py == b.py && a.pz == b.pz && a.qx == b.qx && a.qy == b.qy && a.qz == b.qz && a.qw == b.qw;
}

bool sameJointPivot( const SpawnBody& a, const SpawnBody& b )
{
	return a.jointEnabled == b.jointEnabled && a.jointPivotX == b.jointPivotX && a.jointPivotY == b.jointPivotY &&
	       a.jointPivotZ == b.jointPivotZ;
}

b3Quat quatFromDef( const SpawnBody& def )
{
	b3Quat q;
	q.v.x = 0.0f;
	q.v.y = 0.0f;
	q.v.z = 0.0f;
	q.s = 1.0f;

	float lengthSq = def.qx * def.qx + def.qy * def.qy + def.qz * def.qz + def.qw * def.qw;
	if ( lengthSq > 0.0001f )
	{
		float inv = 1.0f / sqrtf( lengthSq );
		q.v.x = def.qx * inv;
		q.v.y = def.qy * inv;
		q.v.z = def.qz * inv;
		q.s = def.qw * inv;
	}
	return q;
}

void setBodyTransformFromDef( b3BodyId bodyId, const SpawnBody& def )
{
	b3Quat q = quatFromDef( def );

	if ( def.type == 1 )
	{
		// Kinematic bodies should be driven by a target transform so Box3D can
		// derive velocities and produce stable contacts with dynamic bodies.
		// advance() re-targets them before every step, so the velocity decays
		// to zero once the target is reached instead of persisting forever.
		b3WorldTransform target = b3WorldTransform_identity;
		target.p = b3Pos{ def.px, def.py, def.pz };
		target.q = q;
		b3Body_SetTargetTransform( bodyId, target, (float)kFixedTimeStep, true );
	}
	else
	{
		b3Body_SetTransform( bodyId, b3Pos{ def.px, def.py, def.pz }, q );
		b3Body_SetAwake( bodyId, true );
	}
}

// Live material tweaks must not recreate bodies (that would respawn them);
// box3d lets friction/restitution/density change on existing shapes.
void applyMaterialToBody( b3BodyId bodyId, const SpawnBody& def )
{
	b3ShapeId shapes[8];
	int count = b3Body_GetShapes( bodyId, shapes, 8 );
	for ( int s = 0; s < count; ++s )
	{
		b3Shape_SetFriction( shapes[s], def.friction );
		b3Shape_SetRestitution( shapes[s], def.restitution );
		b3Shape_SetDensity( shapes[s], def.density, false );
	}
	b3Body_ApplyMassFromShapes( bodyId );
}

void createBodyFromDef( b3WorldId world, const SpawnBody& def, std::vector<b3BodyId>& outBodies,
						std::vector<b3MeshData*>& outMeshes )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = def.type == 0 ? b3_staticBody : ( def.type == 1 ? b3_kinematicBody : b3_dynamicBody );
	bodyDef.position = b3Pos{ def.px, def.py, def.pz };
	bodyDef.isBullet = def.bullet && bodyDef.type == b3_dynamicBody;

	float lengthSq = def.qx * def.qx + def.qy * def.qy + def.qz * def.qz + def.qw * def.qw;
	if ( lengthSq > 0.0001f )
	{
		float inv = 1.0f / sqrtf( lengthSq );
		b3Quat q;
		q.v.x = def.qx * inv;
		q.v.y = def.qy * inv;
		q.v.z = def.qz * inv;
		q.s = def.qw * inv;
		bodyDef.rotation = q;
	}

	b3BodyId bodyId = b3CreateBody( world, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = def.density;
	shapeDef.baseMaterial.friction = def.friction;
	shapeDef.baseMaterial.restitution = def.restitution;

	// Sizes are full extents; box3d primitives want half extents and radii.
	constexpr float kMinSize = 0.001f;
	float sizeX = def.sizeX > kMinSize ? def.sizeX : kMinSize;
	float sizeY = def.sizeY > kMinSize ? def.sizeY : kMinSize;
	float sizeZ = def.sizeZ > kMinSize ? def.sizeZ : kMinSize;

	switch ( def.shape )
	{
		case 1: // sphere, x = diameter
		{
			b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f * sizeX };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			break;
		}
		case 3: // convex hull from points
		{
			int pointCount = (int)def.hullPoints.size() / 3;
			if ( pointCount >= 4 )
			{
				b3HullData* hull = b3CreateHull( (const b3Vec3*)def.hullPoints.data(), pointCount, kHullVertexBudget );
				if ( hull != nullptr )
				{
					// b3CreateHullShape clones into the world hull database,
					// so our copy can die right after
					b3CreateHullShape( bodyId, &shapeDef, hull );
					b3DestroyHull( hull );
					break;
				}
			}
			// degenerate input: fall through to a box of the given size
			b3BoxHull boxHull = b3MakeBoxHull( 0.5f * sizeX, 0.5f * sizeY, 0.5f * sizeZ );
			b3CreateHullShape( bodyId, &shapeDef, &boxHull.base );
			break;
		}
		case 2: // capsule along Y, x = diameter, y = total height including caps
		{
			float radius = 0.5f * sizeX;
			float half = 0.5f * sizeY - radius;
			half = half > 0.0f ? half : 0.001f;
			b3Capsule capsule = { { 0.0f, -half, 0.0f }, { 0.0f, half, 0.0f }, radius };
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			break;
		}
		case 4: // exact triangle mesh (concave OK); static/kinematic only
		{
			// Dynamic bodies cannot use mesh shapes (engine limit): degrade to
			// the convex hull of the same vertices so the sim keeps running.
			if ( def.type != 2 && def.hullPoints.size() >= 9 && def.meshIndices.size() >= 3 )
			{
				// b3CreateMesh copies into a self-contained blob; the const
				// casts are safe, the input buffers are only read.
				b3MeshDef meshDef = {};
				meshDef.vertices = (b3Vec3*)def.hullPoints.data();
				meshDef.vertexCount = (int)def.hullPoints.size() / 3;
				meshDef.indices = (int32_t*)def.meshIndices.data();
				meshDef.triangleCount = (int)def.meshIndices.size() / 3;
				meshDef.identifyEdges = true;
				// TD geometry often carries duplicated points (Facet, merged
				// primitives). Edge adjacency works on shared indices, so
				// weld first or bodies squeeze through the contact cracks at
				// unshared edges.
				meshDef.weldVertices = true;
				meshDef.weldTolerance = 1e-4f;

				b3MeshData* meshData = b3CreateMesh( &meshDef, nullptr, 0 );
				if ( meshData != nullptr )
				{
					b3CreateMeshShape( bodyId, &shapeDef, meshData, b3Vec3{ 1.0f, 1.0f, 1.0f } );
					outMeshes.push_back( meshData );
					break;
				}
			}

			int pointCount = (int)def.hullPoints.size() / 3;
			if ( pointCount >= 4 )
			{
				b3HullData* hull = b3CreateHull( (const b3Vec3*)def.hullPoints.data(), pointCount, kHullVertexBudget );
				if ( hull != nullptr )
				{
					b3CreateHullShape( bodyId, &shapeDef, hull );
					b3DestroyHull( hull );
					break;
				}
			}

			b3BoxHull boxHull = b3MakeBoxHull( 0.5f * sizeX, 0.5f * sizeY, 0.5f * sizeZ );
			b3CreateHullShape( bodyId, &shapeDef, &boxHull.base );
			break;
		}
		default: // box
		{
			b3BoxHull hull = b3MakeBoxHull( 0.5f * sizeX, 0.5f * sizeY, 0.5f * sizeZ );
			b3CreateHullShape( bodyId, &shapeDef, &hull.base );
			break;
		}
	}

	outBodies.push_back( bodyId );
}

// Mesh blobs die AFTER the bodies/shapes referencing them.
void destroyGroupMeshes( Group& group )
{
	for ( b3MeshData* mesh : group.meshes )
	{
		if ( mesh != nullptr )
		{
			b3DestroyMesh( mesh );
		}
	}
	group.meshes.clear();
}

void destroyGroupBodies( Group& group )
{
	for ( b3BodyId bodyId : group.bodies )
	{
		if ( B3_IS_NON_NULL( bodyId ) )
		{
			b3DestroyBody( bodyId );
		}
	}
	group.bodies.clear();
	destroyGroupMeshes( group );
}

void createGroupBodies( b3WorldId world, Group& group )
{
	group.bodies.clear();
	destroyGroupMeshes( group );
	group.bodies.reserve( group.defs.size() );
	for ( const SpawnBody& def : group.defs )
	{
		createBodyFromDef( world, def, group.bodies, group.meshes );
	}
}

} // namespace

struct SolverCore::Impl
{
	b3WorldId world = b3_nullWorldId;

	// Static collision mesh. box3d keeps a reference to meshData (not a
	// copy), so it must stay alive while the world exists and be destroyed
	// only after the world.
	b3MeshData* meshData = nullptr;
	std::vector<b3Vec3> meshVertices;
	std::vector<int32_t> meshIndices;
	bool hasMesh = false;

	WorldSettings settings;
	std::map<uint32_t, Group> groups;

	int liveJointCount = 0;

	// Joints owned by Box3D Joint CHOP nodes, keyed by node opId. Resolved
	// bodies and local frames are kept so the node can report its live anchors.
	bool jointsDirty = false;

	struct NodeJoint
	{
		JointSpec spec;
		b3JointId id = b3_nullJointId;
		b3BodyId bodyA = b3_nullBodyId;
		b3BodyId bodyB = b3_nullBodyId;
		b3Transform localA = b3Transform_identity;
		b3Transform localB = b3Transform_identity;
	};
	std::map<uint32_t, std::vector<NodeJoint>> nodeJoints;

	// Hidden static body used as the second body of world-anchored joints.
	b3BodyId worldAnchorBody = b3_nullBodyId;

	// Mesh blobs of groups removed while the world was pending a rebuild:
	// their shapes may still be referenced until destroyWorld runs.
	std::vector<b3MeshData*> orphanMeshes;

	bool dirty = true;
	double accumulator = 0.0;
	int64_t stepCount = 0;

	void destroyWorld()
	{
		if ( B3_IS_NON_NULL( world ) )
		{
			b3DestroyWorld( world );
			world = b3_nullWorldId;
		}
		worldAnchorBody = b3_nullBodyId;
		for ( auto& entry : nodeJoints )
		{
			for ( NodeJoint& nj : entry.second )
			{
				nj.id = b3_nullJointId;
			}
		}
		liveJointCount = 0;
		jointsDirty = true;
		// Meshes can only die after the world (their mesh shapes) is gone.
		for ( auto& entry : groups )
		{
			entry.second.bodies.clear();
			destroyGroupMeshes( entry.second );
		}
		for ( b3MeshData* mesh : orphanMeshes )
		{
			if ( mesh != nullptr )
			{
				b3DestroyMesh( mesh );
			}
		}
		orphanMeshes.clear();
		if ( meshData != nullptr )
		{
			b3DestroyMesh( meshData );
			meshData = nullptr;
		}
	}

	void rebuild()
	{
		destroyWorld();

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.gravity = b3Vec3{ settings.gravityX + settings.accelX, settings.gravityY + settings.accelY,
								  settings.gravityZ + settings.accelZ };
		worldDef.workerCount = settings.workerCount > 1 ? settings.workerCount : 1;
		worldDef.enableSleep = settings.sleep;
		world = b3CreateWorld( &worldDef );

		// Shapeless static body: the "other side" of world-anchored joints
		{
			b3BodyDef anchorDef = b3DefaultBodyDef();
			worldAnchorBody = b3CreateBody( world, &anchorDef );
		}

		if ( settings.ground )
		{
			// Static ground slab, top surface at y = 0
			float half = 0.5f * settings.groundSize;
			b3BodyDef groundDef = b3DefaultBodyDef();
			groundDef.position = b3Pos{ 0.0f, -1.0f, 0.0f };
			b3BodyId groundId = b3CreateBody( world, &groundDef );

			b3BoxHull groundBox = b3MakeBoxHull( half, 1.0f, half );
			b3ShapeDef groundShapeDef = b3DefaultShapeDef();
			b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );
		}

		if ( settings.container )
		{
			float half = 0.5f * settings.groundSize;
			float halfHeight = 0.5f * ( settings.wallHeight > 0.01f ? settings.wallHeight : 0.01f );
			float thickness = settings.wallThickness > 0.01f ? settings.wallThickness : 0.01f;
			float halfThickness = 0.5f * thickness;

			auto createWall = [&]( float px, float py, float pz, float hx, float hy, float hz ) {
				b3BodyDef wallDef = b3DefaultBodyDef();
				wallDef.position = b3Pos{ px, py, pz };
				b3BodyId wallId = b3CreateBody( world, &wallDef );

				b3ShapeDef wallShapeDef = b3DefaultShapeDef();
				b3BoxHull wallHull = b3MakeBoxHull( hx, hy, hz );
				b3CreateHullShape( wallId, &wallShapeDef, &wallHull.base );
			};

			// Four static walls around the play area, floor top at y=0.
			createWall( 0.0f, halfHeight, half + halfThickness, half + thickness, halfHeight, halfThickness );
			createWall( 0.0f, halfHeight, -( half + halfThickness ), half + thickness, halfHeight, halfThickness );
			createWall( half + halfThickness, halfHeight, 0.0f, halfThickness, halfHeight, half + thickness );
			createWall( -( half + halfThickness ), halfHeight, 0.0f, halfThickness, halfHeight, half + thickness );
		}

		if ( hasMesh && !meshIndices.empty() )
		{
			b3MeshDef meshDef = {};
			meshDef.vertices = meshVertices.data();
			meshDef.vertexCount = (int)meshVertices.size();
			meshDef.indices = meshIndices.data();
			meshDef.triangleCount = (int)meshIndices.size() / 3;
			meshDef.identifyEdges = true;
			// Weld duplicated TD points so edge adjacency resolves; without it
			// bodies squeeze through contact cracks at unshared edges.
			meshDef.weldVertices = true;
			meshDef.weldTolerance = 1e-4f;

			meshData = b3CreateMesh( &meshDef, nullptr, 0 );
			if ( meshData != nullptr )
			{
				b3BodyDef bodyDef = b3DefaultBodyDef();
				b3BodyId bodyId = b3CreateBody( world, &bodyDef );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateMeshShape( bodyId, &shapeDef, meshData, b3Vec3{ 1.0f, 1.0f, 1.0f } );
			}
		}

		// std::map iterates key-sorted, so respawn order is deterministic
		for ( auto& entry : groups )
		{
			Group& group = entry.second;
			createGroupBodies( world, group );
		}

		accumulator = 0.0;
		stepCount = 0;
		dirty = false;
	}

	const Group* findGroupByPath( const std::string& ref ) const
	{
		for ( const auto& entry : groups )
		{
			if ( pathMatches( entry.second.path, ref ) )
			{
				return &entry.second;
			}
		}
		return nullptr;
	}

	// Resolve a Joints DAT body reference to a live body id.
	b3BodyId resolveBody( const std::string& ref, int index ) const
	{
		const Group* group = findGroupByPath( ref );
		if ( group == nullptr || index < 0 || index >= (int)group->bodies.size() )
		{
			return b3_nullBodyId;
		}
		return group->bodies[index];
	}

	// Creates one joint from a spec, resolving body references. Returns false
	// (leaving outId null) when a reference does not resolve yet — the caller
	// retries on the next resync. Local frames are reported back so node
	// joints can draw their live anchor points.
	bool createJointFromSpec( const JointSpec& spec, b3JointId& outId, b3BodyId& outBodyA, b3BodyId& outBodyB,
							  b3Transform& outLocalA, b3Transform& outLocalB )
	{
		outId = b3_nullJointId;

		b3BodyId bodyA = resolveBody( spec.bodyA, spec.indexA );
		b3BodyId bodyB = spec.bodyB.empty() ? worldAnchorBody : resolveBody( spec.bodyB, spec.indexB );
		if ( !B3_IS_NON_NULL( bodyA ) || !B3_IS_NON_NULL( bodyB ) )
		{
			return false;
		}

		const Group* groupA = findGroupByPath( spec.bodyA );
		const Group* groupB = spec.bodyB.empty() ? nullptr : findGroupByPath( spec.bodyB );
		const bool bodyBIsWorld = spec.bodyB.empty();
		b3Transform xfA = toLocalTransform( bodyA );
		b3Transform xfB = toLocalTransform( bodyB );
		b3Vec3 jointA = b3Vec3{ 0.0f, 0.0f, 0.0f };
		b3Vec3 jointB = b3Vec3{ 0.0f, 0.0f, 0.0f };
		if ( groupA != nullptr && spec.indexA >= 0 && spec.indexA < (int)groupA->defs.size() && groupA->defs[spec.indexA].jointEnabled )
		{
			const SpawnBody& def = groupA->defs[spec.indexA];
			jointA = b3Vec3{ def.jointPivotX, def.jointPivotY, def.jointPivotZ };
		}
		if ( groupB != nullptr && spec.indexB >= 0 && spec.indexB < (int)groupB->defs.size() && groupB->defs[spec.indexB].jointEnabled )
		{
			const SpawnBody& def = groupB->defs[spec.indexB];
			jointB = b3Vec3{ def.jointPivotX, def.jointPivotY, def.jointPivotZ };
		}
		b3Vec3 worldPivotA = b3Add( xfA.p, b3RotateVector( xfA.q, jointA ) );
		b3Vec3 worldPivotB = bodyBIsWorld ? worldPivotA : b3Add( xfB.p, b3RotateVector( xfB.q, jointB ) );

		// One pivot frame per body, each at that body's OWN local pivot; the
		// z-axis is the hinge / cone axis. The joint constrains both frames to
		// coincide, so when the pivots are apart at creation the solver pulls
		// the bodies together until the pivots meet (Bullet-style). Using a
		// single shared frame (e.g. the midpoint) instead would freeze the
		// spawn separation into the constraint as a rigid offset.
		b3Transform pivotA;
		pivotA.q = quatFromZAxis( spec.axisX, spec.axisY, spec.axisZ );
		pivotA.p = worldPivotA;
		b3Transform pivotB = pivotA;
		pivotB.p = worldPivotB;

		b3Transform localA = b3InvMulTransforms( xfA, pivotA );
		b3Transform localB = b3InvMulTransforms( xfB, pivotB );

		b3JointId jointId = b3_nullJointId;
		switch ( spec.type )
		{
			case 0: // distance: attach at each body origin
			{
				b3DistanceJointDef def = b3DefaultDistanceJointDef();
				def.base.bodyIdA = bodyA;
				def.base.bodyIdB = bodyB;
				if ( spec.bodyB.empty() )
				{
					// The world anchor body sits at the origin, so its local
					// frame IS world space: pin the rope at the pivot point.
					localB.p = pivotA.p;
				}
				def.base.localFrameA = localA;
				def.base.localFrameB = localB;
				def.base.collideConnected = spec.collideConnected;

				b3Vec3 worldA = b3Add( xfA.p, b3RotateVector( xfA.q, localA.p ) );
				b3Vec3 worldB = b3Add( xfB.p, b3RotateVector( xfB.q, localB.p ) );
				float dx = worldB.x - worldA.x;
				float dy = worldB.y - worldA.y;
				float dz = worldB.z - worldA.z;
				float current = sqrtf( dx * dx + dy * dy + dz * dz );
				def.length = spec.length >= 0.0f ? spec.length : current;

				if ( spec.hertz > 0.0f )
				{
					def.enableSpring = true;
					def.hertz = spec.hertz;
					def.dampingRatio = spec.dampingRatio;
				}
				if ( spec.enableLimit )
				{
					def.enableLimit = true;
					def.minLength = spec.minLength;
					def.maxLength = spec.maxLength;
				}
				if ( spec.enableMotor )
				{
					def.enableMotor = true;
					def.motorSpeed = spec.motorSpeed;
					def.maxMotorForce = spec.maxMotorForce;
				}
				jointId = b3CreateDistanceJoint( world, &def );
				break;
			}
			case 1: // spherical: cone on frame A z-axis, twist on frame B z-axis
			{
				b3SphericalJointDef def = b3DefaultSphericalJointDef();
				def.base.bodyIdA = bodyA;
				def.base.bodyIdB = bodyB;
				def.base.localFrameA = localA;
				def.base.localFrameB = localB;
				def.base.collideConnected = spec.collideConnected;

				if ( spec.hertz > 0.0f )
				{
					def.enableSpring = true;
					def.hertz = spec.hertz;
					def.dampingRatio = spec.dampingRatio;
				}
				if ( spec.enableConeLimit )
				{
					def.enableConeLimit = true;
					def.coneAngle = spec.coneAngle;
				}
				if ( spec.enableLimit )
				{
					def.enableTwistLimit = true;
					def.lowerTwistAngle = spec.lowerAngle;
					def.upperTwistAngle = spec.upperAngle;
				}
				jointId = b3CreateSphericalJoint( world, &def );
				break;
			}
			case 2: // revolute: hinge on the joint frame z-axis
			{
				b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
				def.base.bodyIdA = bodyA;
				def.base.bodyIdB = bodyB;
				def.base.localFrameA = localA;
				def.base.localFrameB = localB;
				def.base.collideConnected = spec.collideConnected;

				if ( spec.hertz > 0.0f )
				{
					def.enableSpring = true;
					def.hertz = spec.hertz;
					def.dampingRatio = spec.dampingRatio;
				}
				if ( spec.enableLimit )
				{
					def.enableLimit = true;
					def.lowerAngle = spec.lowerAngle;
					def.upperAngle = spec.upperAngle;
				}
				if ( spec.enableMotor )
				{
					def.enableMotor = true;
					def.motorSpeed = spec.motorSpeed;
					def.maxMotorTorque = spec.maxMotorForce;
				}
				jointId = b3CreateRevoluteJoint( world, &def );
				break;
			}
			case 3: // weld
			{
				b3WeldJointDef def = b3DefaultWeldJointDef();
				def.base.bodyIdA = bodyA;
				def.base.bodyIdB = bodyB;
				def.base.localFrameA = localA;
				def.base.localFrameB = localB;
				def.base.collideConnected = spec.collideConnected;
				def.linearHertz = spec.hertz;
				def.angularHertz = spec.hertz;
				def.linearDampingRatio = spec.dampingRatio > 0.0f ? spec.dampingRatio : 1.0f;
				def.angularDampingRatio = def.linearDampingRatio;
				jointId = b3CreateWeldJoint( world, &def );
				break;
			}
			default:
				return false;
		}

		if ( !B3_IS_NON_NULL( jointId ) )
		{
			return false;
		}

		outId = jointId;
		outBodyA = bodyA;
		outBodyB = bodyB;
		outLocalA = localA;
		outLocalB = localB;
		return true;
	}

	// Destroy and recreate every Joint SOP node joint.
	// Runs before stepping whenever groups or the world changed — box3d
	// destroys joints attached to destroyed bodies, so ids can silently die
	// under us and per-joint patching is not worth the bookkeeping here.
	void syncJoints()
	{
		for ( auto& entry : nodeJoints )
		{
			for ( NodeJoint& nj : entry.second )
			{
				if ( B3_IS_NON_NULL( nj.id ) && b3Joint_IsValid( nj.id ) )
				{
					b3DestroyJoint( nj.id, true );
				}
				nj.id = b3_nullJointId;
			}
		}

		liveJointCount = 0;

		if ( !B3_IS_NON_NULL( world ) )
		{
			jointsDirty = false;
			return;
		}

		for ( auto& entry : nodeJoints )
		{
			for ( NodeJoint& nj : entry.second )
			{
				if ( createJointFromSpec( nj.spec, nj.id, nj.bodyA, nj.bodyB, nj.localA, nj.localB ) )
				{
					++liveJointCount;
				}
			}
		}

		jointsDirty = false;
	}

	// b3Body_SetTargetTransform only sets velocities (target - current) / dt;
	// box3d never damps kinematic bodies, so without a fresh target every step
	// they keep the last velocity and drift forever once upstream animation
	// stops. Re-target from the defs before each step: at the target this
	// yields zero velocity and the body can go back to sleep. Sleeping bodies
	// are skipped by the solver, so a kinematic that dozed off while idle must
	// be woken explicitly when its target moves again — with wake always false
	// a resumed upstream animation would never move it (nor anything resting
	// on or jointed to it). Wake only on real movement so idle kinematics can
	// keep sleeping.
	void retargetKinematicBodies()
	{
		for ( auto& entry : groups )
		{
			Group& group = entry.second;
			size_t count = group.bodies.size() < group.defs.size() ? group.bodies.size() : group.defs.size();
			for ( size_t i = 0; i < count; ++i )
			{
				const SpawnBody& def = group.defs[i];
				if ( def.type != 1 || !B3_IS_NON_NULL( group.bodies[i] ) )
				{
					continue;
				}

				b3WorldTransform target = b3WorldTransform_identity;
				target.p = b3Pos{ def.px, def.py, def.pz };
				target.q = quatFromDef( def );

				bool wake = false;
				if ( !b3Body_IsAwake( group.bodies[i] ) )
				{
					b3Pos p = b3Body_GetPosition( group.bodies[i] );
					b3Quat q = b3Body_GetRotation( group.bodies[i] );
					constexpr float kPosEps = 1e-5f;
					constexpr float kRotEps = 1e-5f;
					// q and -q are the same rotation; align signs before diffing.
					float dot = q.v.x * target.q.v.x + q.v.y * target.q.v.y + q.v.z * target.q.v.z + q.s * target.q.s;
					float sign = dot < 0.0f ? -1.0f : 1.0f;
					float dq = fabsf( sign * q.v.x - target.q.v.x ) + fabsf( sign * q.v.y - target.q.v.y ) +
							   fabsf( sign * q.v.z - target.q.v.z ) + fabsf( sign * q.s - target.q.s );
					wake = fabsf( (float)( p.x - target.p.x ) ) > kPosEps ||
						   fabsf( (float)( p.y - target.p.y ) ) > kPosEps ||
						   fabsf( (float)( p.z - target.p.z ) ) > kPosEps || dq > kRotEps;
				}
				b3Body_SetTargetTransform( group.bodies[i], target, (float)kFixedTimeStep, wake );
			}
		}
	}
};

SolverCore::SolverCore() : myImpl( new Impl )
{
}

SolverCore::~SolverCore()
{
	myImpl->destroyWorld();
	delete myImpl;
}

void SolverCore::setWorldSettings( const WorldSettings& settings )
{
	Impl* m = myImpl;
	if ( settings.ground != m->settings.ground || settings.groundSize != m->settings.groundSize ||
		 settings.container != m->settings.container || settings.wallHeight != m->settings.wallHeight ||
		 settings.wallThickness != m->settings.wallThickness )
	{
		m->dirty = true;
	}
	m->settings = settings;

	if ( B3_IS_NON_NULL( m->world ) && !m->dirty )
	{
		b3World_SetGravity( m->world,
			b3Vec3{ settings.gravityX + settings.accelX, settings.gravityY + settings.accelY,
					 settings.gravityZ + settings.accelZ } );
		b3World_SetWorkerCount( m->world, settings.workerCount > 1 ? settings.workerCount : 1 );
		b3World_EnableSleeping( m->world, settings.sleep );
	}
}

void SolverCore::setCollisionMesh( const float* vertices, int vertexCount, const int32_t* indices, int indexCount )
{
	Impl* m = myImpl;
	m->meshVertices.clear();
	m->meshVertices.reserve( vertexCount );
	for ( int i = 0; i < vertexCount; ++i )
	{
		m->meshVertices.push_back( b3Vec3{ vertices[3 * i], vertices[3 * i + 1], vertices[3 * i + 2] } );
	}
	m->meshIndices.assign( indices, indices + indexCount );
	m->hasMesh = true;
	m->dirty = true;
}

void SolverCore::clearCollisionMesh()
{
	Impl* m = myImpl;
	if ( m->hasMesh )
	{
		m->meshVertices.clear();
		m->meshIndices.clear();
		m->hasMesh = false;
		m->dirty = true;
	}
}

void SolverCore::setGroup( uint32_t groupKey, std::vector<SpawnBody> defs )
{
	Impl* m = myImpl;
	auto it = m->groups.find( groupKey );

	// Fast path: world is live. Update only this group.
	if ( it != m->groups.end() && B3_IS_NON_NULL( m->world ) && !m->dirty )
	{
		Group& group = it->second;
		if ( group.bodies.size() == group.defs.size() && defs.size() == group.defs.size() )
		{
			bool compatible = true;
			for ( size_t i = 0; i < defs.size(); ++i )
			{
				if ( !sameShapeAndType( group.defs[i], defs[i] ) )
				{
					compatible = false;
					break;
				}
			}

			if ( compatible )
			{
				for ( size_t i = 0; i < defs.size(); ++i )
				{
					if ( !sameMaterial( group.defs[i], defs[i] ) )
					{
						applyMaterialToBody( group.bodies[i], defs[i] );
					}

					// Spawn pose changes drive static/kinematic bodies. Dynamic
					// bodies only take their spawn pose at creation or on a world
					// rebuild — re-applying it here would teleport them on every
					// cook of an animated spawn SOP and the sim would never run.
					if ( defs[i].type != 2 && !samePose( group.defs[i], defs[i] ) )
					{
						setBodyTransformFromDef( group.bodies[i], defs[i] );
					}

					if ( !sameJointPivot( group.defs[i], defs[i] ) )
					{
						m->jointsDirty = true;
					}
				}

				group.defs = std::move( defs );
				return;
			}
		}

		// Incompatible (shape/size/type/count/hull changed): recreate only this
		// group's bodies in the current world, keep everyone else running.
		// Dynamic bodies whose spawn pose is unchanged keep their simulated
		// pose and velocity, so a live shape/size tweak does not reset the sim.
		struct PreservedState
		{
			bool valid = false;
			b3Pos p;
			b3Quat q;
			b3Vec3 v;
			b3Vec3 w;
		};
		std::vector<PreservedState> preserved;
		if ( group.bodies.size() == group.defs.size() && defs.size() == group.defs.size() )
		{
			preserved.resize( defs.size() );
			for ( size_t i = 0; i < defs.size(); ++i )
			{
				if ( defs[i].type == 2 && group.defs[i].type == 2 && samePose( group.defs[i], defs[i] ) &&
					 B3_IS_NON_NULL( group.bodies[i] ) )
				{
					preserved[i].valid = true;
					preserved[i].p = b3Body_GetPosition( group.bodies[i] );
					preserved[i].q = b3Body_GetRotation( group.bodies[i] );
					preserved[i].v = b3Body_GetLinearVelocity( group.bodies[i] );
					preserved[i].w = b3Body_GetAngularVelocity( group.bodies[i] );
				}
			}
		}

		destroyGroupBodies( group );
		group.defs = std::move( defs );
		createGroupBodies( m->world, group );
		m->jointsDirty = true; // box3d killed any joints attached to the old bodies

		for ( size_t i = 0; i < preserved.size() && i < group.bodies.size(); ++i )
		{
			if ( preserved[i].valid )
			{
				b3Body_SetTransform( group.bodies[i], preserved[i].p, preserved[i].q );
				b3Body_SetLinearVelocity( group.bodies[i], preserved[i].v );
				b3Body_SetAngularVelocity( group.bodies[i], preserved[i].w );
			}
		}
		return;
	}

	if ( it == m->groups.end() && B3_IS_NON_NULL( m->world ) && !m->dirty )
	{
		Group& group = m->groups[groupKey];
		group.defs = std::move( defs );
		createGroupBodies( m->world, group );
		m->jointsDirty = true; // unresolved joint rows may reference this group
		return;
	}

	Group& group = m->groups[groupKey];
	group.defs = std::move( defs );
	group.bodies.clear();
	m->dirty = true;
}

void SolverCore::removeGroup( uint32_t groupKey )
{
	Impl* m = myImpl;
	auto it = m->groups.find( groupKey );
	if ( it != m->groups.end() )
	{
		// Fast path: remove only this group's bodies when the world is live.
		if ( B3_IS_NON_NULL( m->world ) && !m->dirty )
		{
			destroyGroupBodies( it->second );
			m->groups.erase( it );
			m->jointsDirty = true; // joints attached to those bodies died with them
			return;
		}

		// World rebuild pending: the old world (and its mesh shapes) may
		// still be alive, so park the mesh blobs until destroyWorld runs.
		m->orphanMeshes.insert( m->orphanMeshes.end(), it->second.meshes.begin(), it->second.meshes.end() );
		it->second.meshes.clear();
		m->groups.erase( it );
		m->dirty = true;
	}
}

bool SolverCore::hasGroup( uint32_t groupKey ) const
{
	return myImpl->groups.find( groupKey ) != myImpl->groups.end();
}

void SolverCore::setGroupPath( uint32_t groupKey, const char* path )
{
	Impl* m = myImpl;
	auto it = m->groups.find( groupKey );
	if ( it == m->groups.end() || path == nullptr )
	{
		return;
	}
	if ( it->second.path != path )
	{
		it->second.path = path;
		m->jointsDirty = true; // joint rows referencing this path can resolve now
	}
}

int SolverCore::activeJointCount() const
{
	return myImpl->liveJointCount;
}

void SolverCore::setJointNodeList( uint32_t ownerKey, const std::vector<JointSpec>& specs )
{
	Impl* m = myImpl;
	auto it = m->nodeJoints.find( ownerKey );

	if ( specs.empty() )
	{
		removeJointNode( ownerKey );
		return;
	}

	if ( it != m->nodeJoints.end() )
	{
		const std::vector<Impl::NodeJoint>& current = it->second;
		if ( current.size() == specs.size() )
		{
			bool same = true;
			for ( size_t i = 0; i < specs.size(); ++i )
			{
				if ( !sameJointSpec( current[i].spec, specs[i] ) )
				{
					same = false;
					break;
				}
			}
			if ( same )
			{
				return;
			}
		}
	}

	std::vector<Impl::NodeJoint>& joints = m->nodeJoints[ownerKey];
	joints.clear();
	joints.resize( specs.size() );
	for ( size_t i = 0; i < specs.size(); ++i )
	{
		joints[i].spec = specs[i];
	}
	m->jointsDirty = true;
}

void SolverCore::removeJointNode( uint32_t ownerKey )
{
	Impl* m = myImpl;
	auto it = m->nodeJoints.find( ownerKey );
	if ( it == m->nodeJoints.end() )
	{
		return;
	}

	for ( const Impl::NodeJoint& nj : it->second )
	{
		if ( B3_IS_NON_NULL( nj.id ) && b3Joint_IsValid( nj.id ) )
		{
			b3DestroyJoint( nj.id, true );
			if ( m->liveJointCount > 0 )
			{
				--m->liveJointCount;
			}
		}
	}
	m->nodeJoints.erase( it );
}

bool SolverCore::getJointAnchors( uint32_t ownerKey, int jointIndex, float outA[3], float outB[3] ) const
{
	const Impl* m = myImpl;
	auto it = m->nodeJoints.find( ownerKey );
	if ( it == m->nodeJoints.end() )
	{
		return false;
	}
	if ( jointIndex < 0 || jointIndex >= (int)it->second.size() )
	{
		return false;
	}

	const Impl::NodeJoint& nj = it->second[(size_t)jointIndex];
	if ( !B3_IS_NON_NULL( nj.id ) || !b3Joint_IsValid( nj.id ) )
	{
		return false;
	}

	b3Transform xfA = toLocalTransform( nj.bodyA );
	b3Transform xfB = toLocalTransform( nj.bodyB );
	b3Vec3 a = b3Add( b3RotateVector( xfA.q, nj.localA.p ), xfA.p );
	b3Vec3 b = b3Add( b3RotateVector( xfB.q, nj.localB.p ), xfB.p );
	outA[0] = a.x;
	outA[1] = a.y;
	outA[2] = a.z;
	outB[0] = b.x;
	outB[1] = b.y;
	outB[2] = b.z;
	return true;
}

int SolverCore::groupCount() const
{
	return (int)myImpl->groups.size();
}

void SolverCore::requestRebuild()
{
	myImpl->dirty = true;
}

void SolverCore::advance( double dtSeconds, bool simulate )
{
	Impl* m = myImpl;
	if ( m->dirty )
	{
		m->rebuild();
	}

	if ( m->jointsDirty )
	{
		m->syncJoints();
	}

	if ( !simulate )
	{
		return;
	}

	// Negative deltas (timeline scrubbed backwards) must not drain the
	// accumulator, or the sim stalls until real time pays the debt back.
	if ( dtSeconds > 0.0 )
	{
		m->accumulator += dtSeconds;
	}
	int maxSteps = m->settings.maxStepsPerCook > 0 ? m->settings.maxStepsPerCook : 1;

	int steps = 0;
	while ( m->accumulator >= kFixedTimeStep && steps < maxSteps )
	{
		m->retargetKinematicBodies();
		b3World_Step( m->world, (float)kFixedTimeStep, m->settings.subSteps );
		m->accumulator -= kFixedTimeStep;
		++steps;
		++m->stepCount;
	}

	// Drop time we refused to simulate, otherwise it piles up forever.
	if ( m->accumulator >= kFixedTimeStep )
	{
		m->accumulator = 0.0;
	}
}

int SolverCore::getGroupBodyCount( uint32_t groupKey ) const
{
	auto it = myImpl->groups.find( groupKey );
	if ( it == myImpl->groups.end() )
	{
		return 0;
	}
	const Group& group = it->second;
	return (int)( group.bodies.empty() ? group.defs.size() : group.bodies.size() );
}

int SolverCore::getGroupTransforms( uint32_t groupKey, BodyTransform* out, int capacity ) const
{
	auto it = myImpl->groups.find( groupKey );
	if ( it == myImpl->groups.end() )
	{
		return 0;
	}

	const Group& group = it->second;

	if ( !group.bodies.empty() )
	{
		int count = (int)group.bodies.size() < capacity ? (int)group.bodies.size() : capacity;
		for ( int i = 0; i < count; ++i )
		{
			b3Pos p = b3Body_GetPosition( group.bodies[i] );
			b3Quat q = b3Body_GetRotation( group.bodies[i] );
			out[i] = BodyTransform{ (float)p.x, (float)p.y, (float)p.z, q.v.x, q.v.y, q.v.z, q.s };
		}
		return count;
	}

	// Not built yet (world rebuild pending): report the spawn poses
	int count = (int)group.defs.size() < capacity ? (int)group.defs.size() : capacity;
	for ( int i = 0; i < count; ++i )
	{
		const SpawnBody& d = group.defs[i];
		out[i] = BodyTransform{ d.px, d.py, d.pz, d.qx, d.qy, d.qz, d.qw };
	}
	return count;
}

int SolverCore::getGroupStates( uint32_t groupKey, BodyState* out, int capacity ) const
{
	auto it = myImpl->groups.find( groupKey );
	if ( it == myImpl->groups.end() )
	{
		return 0;
	}

	const Group& group = it->second;

	if ( !group.bodies.empty() )
	{
		int count = (int)group.bodies.size() < capacity ? (int)group.bodies.size() : capacity;
		for ( int i = 0; i < count; ++i )
		{
			b3BodyId bodyId = group.bodies[i];
			b3Pos p = b3Body_GetPosition( bodyId );
			b3Quat q = b3Body_GetRotation( bodyId );
			b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
			b3Vec3 w = b3Body_GetAngularVelocity( bodyId );
			out[i] = BodyState{ (float)p.x, (float)p.y, (float)p.z, q.v.x, q.v.y, q.v.z, q.s,
								v.x,		v.y,		v.z,		w.x,   w.y,   w.z,
								b3Body_IsAwake( bodyId ) ? 1.0f : 0.0f };
		}
		return count;
	}

	// Not built yet: spawn poses with zero velocities
	int count = (int)group.defs.size() < capacity ? (int)group.defs.size() : capacity;
	for ( int i = 0; i < count; ++i )
	{
		const SpawnBody& d = group.defs[i];
		out[i] = BodyState{ d.px, d.py, d.pz, d.qx, d.qy, d.qz, d.qw, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	}
	return count;
}

int SolverCore::totalBodyCount() const
{
	int total = 0;
	for ( const auto& entry : myImpl->groups )
	{
		const Group& group = entry.second;
		total += (int)( group.bodies.empty() ? group.defs.size() : group.bodies.size() );
	}
	return total;
}

int SolverCore::awakeBodyCount() const
{
	return B3_IS_NON_NULL( myImpl->world ) ? b3World_GetAwakeBodyCount( myImpl->world ) : 0;
}

int64_t SolverCore::stepCount() const
{
	return myImpl->stepCount;
}

// ---------------- Registry ----------------

namespace
{
std::map<uint32_t, SolverCore*>& registryMap()
{
	static std::map<uint32_t, SolverCore*> theMap;
	return theMap;
}
} // namespace

SolverCore* Registry::getOrCreate( uint32_t solverOpId )
{
	auto& map = registryMap();
	auto it = map.find( solverOpId );
	if ( it != map.end() )
	{
		return it->second;
	}
	SolverCore* core = new SolverCore();
	map[solverOpId] = core;
	return core;
}

SolverCore* Registry::find( uint32_t solverOpId )
{
	auto& map = registryMap();
	auto it = map.find( solverOpId );
	return it != map.end() ? it->second : nullptr;
}

void Registry::destroy( uint32_t solverOpId )
{
	auto& map = registryMap();
	auto it = map.find( solverOpId );
	if ( it != map.end() )
	{
		delete it->second;
		map.erase( it );
	}
}

} // namespace tdb3

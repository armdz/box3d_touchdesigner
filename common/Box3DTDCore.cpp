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

	// advance() counter value of the owner node's last cook (heartbeat).
	uint64_t lastTouch = 0;
};

// Body identity for contact events, packed into the body's userData pointer:
// owner group key (client node opId) in the high 32 bits, index+1 in the low
// 32 so a real body ref is never null. World statics (ground/walls/collision
// mesh/world anchor) keep a null userData and read back as group 0.
void* packBodyRef( uint32_t groupKey, int index )
{
	return (void*)( ( (uint64_t)groupKey << 32 ) | (uint32_t)( index + 1 ) );
}

void unpackBodyRef( void* userData, uint32_t& groupKey, int& index )
{
	uint64_t v = (uint64_t)(uintptr_t)userData;
	if ( v == 0 )
	{
		groupKey = 0;
		index = -1;
		return;
	}
	groupKey = (uint32_t)( v >> 32 );
	index = (int)(uint32_t)v - 1;
}

uint64_t bodyRefKey( uint32_t groupKey, int index )
{
	return ( (uint64_t)groupKey << 32 ) | (uint32_t)( index + 1 );
}

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

// Same joint EXCEPT the spring parameters (hertz / dampingRatio / length),
// which box3d can change on a live joint without recreating it. The spring
// enabled state (hertz > 0) must still match: toggling a spring on/off changes
// how the joint was built, so that case falls back to a full recreate.
bool sameJointStructure( const JointSpec& a, const JointSpec& b )
{
	return a.type == b.type && a.bodyA == b.bodyA && a.indexA == b.indexA && a.bodyB == b.bodyB &&
		   a.indexB == b.indexB && a.pivotMode == b.pivotMode && a.anchorX == b.anchorX && a.anchorY == b.anchorY &&
		   a.anchorZ == b.anchorZ && a.axisX == b.axisX && a.axisY == b.axisY && a.axisZ == b.axisZ &&
		   ( a.hertz > 0.0f ) == ( b.hertz > 0.0f ) && a.enableLimit == b.enableLimit && a.minLength == b.minLength &&
		   a.maxLength == b.maxLength && a.lowerAngle == b.lowerAngle && a.upperAngle == b.upperAngle &&
		   a.enableConeLimit == b.enableConeLimit && a.coneAngle == b.coneAngle && a.enableMotor == b.enableMotor &&
		   a.motorSpeed == b.motorSpeed && a.maxMotorForce == b.maxMotorForce &&
		   a.collideConnected == b.collideConnected;
}

// Spring params of these joint types can be updated on a live joint id.
bool jointTypeSupportsLiveSpring( int type )
{
	return type == 0 || type == 1 || type == 2; // distance, spherical, revolute
}

// Push a spec's spring params onto an existing joint without recreating it.
void applyLiveSpringParams( b3JointId id, const JointSpec& spec )
{
	switch ( spec.type )
	{
		case 0: // distance
			b3DistanceJoint_SetSpringHertz( id, spec.hertz );
			b3DistanceJoint_SetSpringDampingRatio( id, spec.dampingRatio );
			if ( spec.length >= 0.0f )
			{
				b3DistanceJoint_SetLength( id, spec.length );
			}
			break;
		case 1: // spherical
			b3SphericalJoint_SetSpringHertz( id, spec.hertz );
			b3SphericalJoint_SetSpringDampingRatio( id, spec.dampingRatio );
			break;
		case 2: // revolute
			b3RevoluteJoint_SetSpringHertz( id, spec.hertz );
			b3RevoluteJoint_SetSpringDampingRatio( id, spec.dampingRatio );
			break;
		default:
			break;
	}
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

// ---- debug wireframe helpers ----

b3Vec3 debugTransformPoint( const b3Transform& xf, const b3Vec3& p )
{
	return b3Add( xf.p, b3RotateVector( xf.q, p ) );
}

void debugPushSegment( std::vector<float>& segments, std::vector<float>& colors, const float rgb[3], const b3Vec3& a,
					   const b3Vec3& b )
{
	segments.push_back( a.x );
	segments.push_back( a.y );
	segments.push_back( a.z );
	segments.push_back( b.x );
	segments.push_back( b.y );
	segments.push_back( b.z );
	colors.push_back( rgb[0] );
	colors.push_back( rgb[1] );
	colors.push_back( rgb[2] );
}

// Any two unit vectors perpendicular to axis (and each other).
void debugBasisFromAxis( const b3Vec3& axis, b3Vec3& u, b3Vec3& v )
{
	b3Vec3 ref = fabsf( axis.y ) < 0.9f ? b3Vec3{ 0.0f, 1.0f, 0.0f } : b3Vec3{ 1.0f, 0.0f, 0.0f };
	u = b3Vec3{ axis.y * ref.z - axis.z * ref.y, axis.z * ref.x - axis.x * ref.z, axis.x * ref.y - axis.y * ref.x };
	float len = sqrtf( u.x * u.x + u.y * u.y + u.z * u.z );
	if ( len < 1e-8f )
	{
		u = b3Vec3{ 1.0f, 0.0f, 0.0f };
	}
	else
	{
		u.x /= len;
		u.y /= len;
		u.z /= len;
	}
	v = b3Vec3{ axis.y * u.z - axis.z * u.y, axis.z * u.x - axis.x * u.z, axis.x * u.y - axis.y * u.x };
}

// Circle of radius r around `center` in the plane spanned by u/v (local
// space), transformed to world.
void debugAppendCircle( std::vector<float>& segments, std::vector<float>& colors, const float rgb[3],
						const b3Transform& xf, const b3Vec3& center, const b3Vec3& u, const b3Vec3& v, float r )
{
	constexpr int kSegments = 16;
	constexpr float kTau = 6.28318530717958647692f;
	b3Vec3 prev = debugTransformPoint(
		xf, b3Vec3{ center.x + r * u.x, center.y + r * u.y, center.z + r * u.z } );
	for ( int i = 1; i <= kSegments; ++i )
	{
		float a = kTau * (float)i / (float)kSegments;
		float c = cosf( a );
		float s = sinf( a );
		b3Vec3 p = debugTransformPoint( xf, b3Vec3{ center.x + r * ( c * u.x + s * v.x ),
													center.y + r * ( c * u.y + s * v.y ),
													center.z + r * ( c * u.z + s * v.z ) } );
		debugPushSegment( segments, colors, rgb, prev, p );
		prev = p;
	}
}

// The wireframe of ONE live shape, exactly as box3d stores it (hulls after
// the vertex-budget simplification, meshes after welding).
void debugAppendShape( std::vector<float>& segments, std::vector<float>& colors, const float rgb[3],
					   const b3Transform& xf, b3ShapeId shapeId )
{
	switch ( b3Shape_GetType( shapeId ) )
	{
		case b3_sphereShape:
		{
			b3Sphere s = b3Shape_GetSphere( shapeId );
			debugAppendCircle( segments, colors, rgb, xf, s.center, b3Vec3{ 1, 0, 0 }, b3Vec3{ 0, 1, 0 }, s.radius );
			debugAppendCircle( segments, colors, rgb, xf, s.center, b3Vec3{ 0, 1, 0 }, b3Vec3{ 0, 0, 1 }, s.radius );
			debugAppendCircle( segments, colors, rgb, xf, s.center, b3Vec3{ 0, 0, 1 }, b3Vec3{ 1, 0, 0 }, s.radius );
			break;
		}
		case b3_capsuleShape:
		{
			b3Capsule c = b3Shape_GetCapsule( shapeId );
			b3Vec3 axis = b3Vec3{ c.center2.x - c.center1.x, c.center2.y - c.center1.y, c.center2.z - c.center1.z };
			float len = sqrtf( axis.x * axis.x + axis.y * axis.y + axis.z * axis.z );
			if ( len > 1e-8f )
			{
				axis.x /= len;
				axis.y /= len;
				axis.z /= len;
			}
			else
			{
				axis = b3Vec3{ 0.0f, 1.0f, 0.0f };
			}
			b3Vec3 u, v;
			debugBasisFromAxis( axis, u, v );
			debugAppendCircle( segments, colors, rgb, xf, c.center1, u, v, c.radius );
			debugAppendCircle( segments, colors, rgb, xf, c.center2, u, v, c.radius );
			const b3Vec3 dirs[4] = { u, b3Vec3{ -u.x, -u.y, -u.z }, v, b3Vec3{ -v.x, -v.y, -v.z } };
			for ( const b3Vec3& d : dirs )
			{
				b3Vec3 a = debugTransformPoint( xf, b3Vec3{ c.center1.x + c.radius * d.x, c.center1.y + c.radius * d.y,
															c.center1.z + c.radius * d.z } );
				b3Vec3 b = debugTransformPoint( xf, b3Vec3{ c.center2.x + c.radius * d.x, c.center2.y + c.radius * d.y,
															c.center2.z + c.radius * d.z } );
				debugPushSegment( segments, colors, rgb, a, b );
			}
			// Axis caps so the hemispheres read in the wireframe.
			b3Vec3 tip1 = debugTransformPoint( xf, b3Vec3{ c.center1.x - c.radius * axis.x,
														   c.center1.y - c.radius * axis.y,
														   c.center1.z - c.radius * axis.z } );
			b3Vec3 tip2 = debugTransformPoint( xf, b3Vec3{ c.center2.x + c.radius * axis.x,
														   c.center2.y + c.radius * axis.y,
														   c.center2.z + c.radius * axis.z } );
			b3Vec3 e1 = debugTransformPoint( xf, c.center1 );
			b3Vec3 e2 = debugTransformPoint( xf, c.center2 );
			debugPushSegment( segments, colors, rgb, e1, tip1 );
			debugPushSegment( segments, colors, rgb, e2, tip2 );
			break;
		}
		case b3_hullShape:
		{
			const b3HullData* hull = b3Shape_GetHull( shapeId );
			if ( hull == nullptr )
			{
				break;
			}
			const b3Vec3* points = b3GetHullPoints( hull );
			const b3HullHalfEdge* edges = b3GetHullEdges( hull );
			if ( points == nullptr || edges == nullptr )
			{
				break;
			}
			for ( int e = 0; e < hull->edgeCount; ++e )
			{
				// Each edge is two half-edges; draw it once.
				if ( e < (int)edges[e].twin )
				{
					const b3Vec3& a = points[edges[e].origin];
					const b3Vec3& b = points[edges[edges[e].twin].origin];
					debugPushSegment( segments, colors, rgb, debugTransformPoint( xf, a ),
									  debugTransformPoint( xf, b ) );
				}
			}
			break;
		}
		case b3_meshShape:
		{
			b3Mesh mesh = b3Shape_GetMesh( shapeId );
			if ( mesh.data == nullptr )
			{
				break;
			}
			const b3Vec3* verts = b3GetMeshVertices( mesh.data );
			const b3MeshTriangle* tris = b3GetMeshTriangles( mesh.data );
			if ( verts == nullptr || tris == nullptr )
			{
				break;
			}
			auto scaled = [&]( int32_t i ) {
				return b3Vec3{ verts[i].x * mesh.scale.x, verts[i].y * mesh.scale.y, verts[i].z * mesh.scale.z };
			};
			for ( int t = 0; t < mesh.data->triangleCount; ++t )
			{
				b3Vec3 a = debugTransformPoint( xf, scaled( tris[t].index1 ) );
				b3Vec3 b = debugTransformPoint( xf, scaled( tris[t].index2 ) );
				b3Vec3 c = debugTransformPoint( xf, scaled( tris[t].index3 ) );
				// Shared edges are drawn once via the index-order filter; a
				// boundary edge with descending indices still needs drawing,
				// but our meshes are welded closed surfaces, so accept the
				// (rare) missing open edge over doubling every interior one.
				if ( tris[t].index1 < tris[t].index2 )
					debugPushSegment( segments, colors, rgb, a, b );
				if ( tris[t].index2 < tris[t].index3 )
					debugPushSegment( segments, colors, rgb, b, c );
				if ( tris[t].index3 < tris[t].index1 )
					debugPushSegment( segments, colors, rgb, c, a );
			}
			break;
		}
		default:
			break;
	}
}

void debugAppendBody( std::vector<float>& segments, std::vector<float>& colors, const float rgb[3], b3BodyId bodyId )
{
	if ( !B3_IS_NON_NULL( bodyId ) )
	{
		return;
	}
	b3Transform xf = toLocalTransform( bodyId );
	// 64 covers compound bodies (one shape per piece).
	b3ShapeId shapes[64];
	int count = b3Body_GetShapes( bodyId, shapes, 64 );
	for ( int s = 0; s < count; ++s )
	{
		debugAppendShape( segments, colors, rgb, xf, shapes[s] );
	}
}

constexpr float kDebugStaticColor[3] = { 0.55f, 0.55f, 0.6f };
constexpr float kDebugKinematicColor[3] = { 1.0f, 0.55f, 0.1f };
constexpr float kDebugAwakeColor[3] = { 0.25f, 0.95f, 0.35f };
constexpr float kDebugAsleepColor[3] = { 0.25f, 0.45f, 1.0f };

// One group's bodies, colored by type/state.
void debugAppendGroup( std::vector<float>& segments, std::vector<float>& colors, const Group& group )
{
	for ( size_t i = 0; i < group.bodies.size(); ++i )
	{
		b3BodyId bodyId = group.bodies[i];
		if ( !B3_IS_NON_NULL( bodyId ) )
		{
			continue;
		}
		int type = i < group.defs.size() ? group.defs[i].type : 2;
		const float* rgb = kDebugStaticColor;
		if ( type == 1 )
		{
			rgb = kDebugKinematicColor;
		}
		else if ( type == 2 )
		{
			rgb = b3Body_IsAwake( bodyId ) ? kDebugAwakeColor : kDebugAsleepColor;
		}
		debugAppendBody( segments, colors, rgb, bodyId );
	}
}

bool sameShapeAndType( const SpawnBody& a, const SpawnBody& b )
{
	if ( a.shape != b.shape || a.sizeX != b.sizeX || a.sizeY != b.sizeY || a.sizeZ != b.sizeZ || a.type != b.type ||
		 a.bullet != b.bullet || a.wallThickness != b.wallThickness || a.openTop != b.openTop )
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

	if ( a.hullPieceCounts.size() != b.hullPieceCounts.size() )
	{
		return false;
	}

	for ( size_t i = 0; i < a.hullPieceCounts.size(); ++i )
	{
		if ( a.hullPieceCounts[i] != b.hullPieceCounts[i] )
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

	// NaN spawn poses (degenerate upstream geometry) must never reach the
	// world; a poisoned static/kinematic transform corrupts every contact.
	float px = std::isfinite( def.px ) ? def.px : 0.0f;
	float py = std::isfinite( def.py ) ? def.py : 0.0f;
	float pz = std::isfinite( def.pz ) ? def.pz : 0.0f;

	if ( def.type == 1 )
	{
		// Kinematic bodies should be driven by a target transform so Box3D can
		// derive velocities and produce stable contacts with dynamic bodies.
		// advance() re-targets them before every step, so the velocity decays
		// to zero once the target is reached instead of persisting forever.
		b3WorldTransform target = b3WorldTransform_identity;
		target.p = b3Pos{ px, py, pz };
		target.q = q;
		b3Body_SetTargetTransform( bodyId, target, (float)kFixedTimeStep, true );
	}
	else
	{
		b3Body_SetTransform( bodyId, b3Pos{ px, py, pz }, q );
		b3Body_SetAwake( bodyId, true );
	}
}

// Live material tweaks must not recreate bodies (that would respawn them);
// box3d lets friction/restitution/density change on existing shapes.
void applyMaterialToBody( b3BodyId bodyId, const SpawnBody& def )
{
	float density = std::isfinite( def.density ) && def.density > 0.0f ? def.density : 0.0f;
	float friction = std::isfinite( def.friction ) && def.friction > 0.0f ? def.friction : 0.0f;
	float restitution = std::isfinite( def.restitution ) && def.restitution > 0.0f ? def.restitution : 0.0f;

	// 64 covers compound bodies (one shape per piece).
	b3ShapeId shapes[64];
	int count = b3Body_GetShapes( bodyId, shapes, 64 );
	for ( int s = 0; s < count; ++s )
	{
		b3Shape_SetFriction( shapes[s], friction );
		b3Shape_SetRestitution( shapes[s], restitution );
		b3Shape_SetDensity( shapes[s], density, false );
	}
	b3Body_ApplyMassFromShapes( bodyId );
}

inline float finiteOr( float v, float fallback )
{
	return std::isfinite( v ) ? v : fallback;
}

void createBodyFromDef( b3WorldId world, const SpawnBody& def, std::vector<b3BodyId>& outBodies,
						std::vector<b3MeshData*>& outMeshes )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = def.type == 0 ? b3_staticBody : ( def.type == 1 ? b3_kinematicBody : b3_dynamicBody );
	// Spawn data can come from arbitrary per-point attributes (noise,
	// divisions by zero upstream). A single NaN position poisons the whole
	// island/world and never heals, so sanitize here — the one choke point
	// every body goes through.
	bodyDef.position = b3Pos{ finiteOr( def.px, 0.0f ), finiteOr( def.py, 0.0f ), finiteOr( def.pz, 0.0f ) };
	bodyDef.isBullet = def.bullet && bodyDef.type == b3_dynamicBody;

	float qx = finiteOr( def.qx, 0.0f );
	float qy = finiteOr( def.qy, 0.0f );
	float qz = finiteOr( def.qz, 0.0f );
	float qw = finiteOr( def.qw, 1.0f );
	float lengthSq = qx * qx + qy * qy + qz * qz + qw * qw;
	if ( lengthSq > 0.0001f && std::isfinite( lengthSq ) )
	{
		float inv = 1.0f / sqrtf( lengthSq );
		b3Quat q;
		q.v.x = qx * inv;
		q.v.y = qy * inv;
		q.v.z = qz * inv;
		q.s = qw * inv;
		bodyDef.rotation = q;
	}

	b3BodyId bodyId = b3CreateBody( world, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	// Attribute-driven materials bypass the parameter clamps; keep them sane.
	float density = finiteOr( def.density, 1.0f );
	float friction = finiteOr( def.friction, 0.6f );
	float restitution = finiteOr( def.restitution, 0.0f );
	shapeDef.density = density > 0.0f ? density : 0.0f;
	shapeDef.baseMaterial.friction = friction > 0.0f ? friction : 0.0f;
	shapeDef.baseMaterial.restitution = restitution > 0.0f ? restitution : 0.0f;
	// Contact/hit events are per-shape flags OR-ed across each pair, so
	// enabling them on group shapes is enough — pairs against the flagless
	// world statics (ground/walls/collision mesh) still report.
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;

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
		case 5: // compound: one convex hull shape per piece on the same body.
		{
			// Multiple shapes on one body is box3d's native composition: mass
			// and inertia come from all pieces, so concave objects modeled in
			// convex-ish parts can be fully dynamic.
			bool any = false;
			int offset = 0;
			for ( int32_t count : def.hullPieceCounts )
			{
				if ( count >= 4 && ( (size_t)( offset + count ) ) * 3 <= def.hullPoints.size() )
				{
					b3HullData* hull =
						b3CreateHull( (const b3Vec3*)def.hullPoints.data() + offset, count, kHullVertexBudget );
					if ( hull != nullptr )
					{
						b3CreateHullShape( bodyId, &shapeDef, hull );
						b3DestroyHull( hull );
						any = true;
					}
				}
				offset += count;
			}
			if ( any )
			{
				break;
			}

			// No usable piece: box of the given size so the body still exists.
			b3BoxHull boxHull = b3MakeBoxHull( 0.5f * sizeX, 0.5f * sizeY, 0.5f * sizeZ );
			b3CreateHullShape( bodyId, &shapeDef, &boxHull.base );
			break;
		}
		case 6: // hollow box / container: thin wall slabs, collision faces INWARD
		{
			// Six (or five, if open-top) thin box slabs inset against the outer
			// surface. Each slab is a solid convex shape, so objects inside the
			// cavity collide against the inner faces and stay contained, while
			// the whole thing can still be static, kinematic or dynamic.
			float hx = 0.5f * sizeX;
			float hy = 0.5f * sizeY;
			float hz = 0.5f * sizeZ;
			float t = def.wallThickness > 0.001f ? def.wallThickness : 0.001f;
			float ht = 0.5f * t;
			// Clamp so walls never overlap past the box center.
			if ( ht > hx * 0.98f )
				ht = hx * 0.98f;
			if ( ht > hy * 0.98f )
				ht = hy * 0.98f;
			if ( ht > hz * 0.98f )
				ht = hz * 0.98f;

			auto addWall = [&]( float whx, float why, float whz, b3Vec3 offset ) {
				b3BoxHull wall = b3MakeOffsetBoxHull( whx, why, whz, offset );
				b3CreateHullShape( bodyId, &shapeDef, &wall.base );
			};

			addWall( ht, hy, hz, b3Vec3{ hx - ht, 0.0f, 0.0f } );  // +X
			addWall( ht, hy, hz, b3Vec3{ -( hx - ht ), 0.0f, 0.0f } ); // -X
			addWall( hx, hy, ht, b3Vec3{ 0.0f, 0.0f, hz - ht } );  // +Z
			addWall( hx, hy, ht, b3Vec3{ 0.0f, 0.0f, -( hz - ht ) } ); // -Z
			addWall( hx, ht, hz, b3Vec3{ 0.0f, -( hy - ht ), 0.0f } ); // -Y (floor)
			if ( !def.openTop )
			{
				addWall( hx, ht, hz, b3Vec3{ 0.0f, hy - ht, 0.0f } ); // +Y (lid)
			}
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

void createGroupBodies( b3WorldId world, uint32_t groupKey, Group& group )
{
	group.bodies.clear();
	destroyGroupMeshes( group );
	group.bodies.reserve( group.defs.size() );
	for ( const SpawnBody& def : group.defs )
	{
		createBodyFromDef( world, def, group.bodies, group.meshes );
	}
	for ( size_t i = 0; i < group.bodies.size(); ++i )
	{
		if ( B3_IS_NON_NULL( group.bodies[i] ) )
		{
			b3Body_SetUserData( group.bodies[i], packBodyRef( groupKey, (int)i ) );
		}
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

	// Force fields owned by Force CHOP nodes, keyed by node opId.
	std::map<uint32_t, std::vector<ForceField>> forceNodes;

	// Hidden static body used as the second body of world-anchored joints.
	b3BodyId worldAnchorBody = b3_nullBodyId;

	// Mesh blobs of groups removed while the world was pending a rebuild:
	// their shapes may still be referenced until destroyWorld runs.
	std::vector<b3MeshData*> orphanMeshes;

	// Ground / container walls / world collision mesh bodies, kept only so
	// the debug wireframe can draw them.
	std::vector<b3BodyId> worldStaticBodies;

	bool dirty = true;
	double accumulator = 0.0;
	int64_t stepCount = 0;

	// Collision events of the last simulating advance(): every fixed step of
	// that advance appends here, and the next simulating advance clears it.
	// Clients read it after the solver's cook (their Solver param dependency
	// guarantees the order), so the buffer is stable for the rest of the frame.
	std::vector<ContactEvent> events;
	// Strongest hit approach speed per body (bodyRefKey) in the last advance,
	// for per-instance "hit" channels.
	std::map<uint64_t, float> hitSpeeds;

	// Client liveness. Bypassed/disabled nodes never cook again and cannot
	// unregister themselves (the SDK has no bypass notification), so advance()
	// drops groups/joints whose owner stopped touching them. Client nodes use
	// cookEveryFrame and re-touch on every cook.
	uint64_t advanceCounter = 0;
	std::map<uint32_t, uint64_t> jointTouch;
	std::map<uint32_t, uint64_t> forceTouch;
	std::map<uint32_t, uint64_t> grabTouch;

	// One live grab constraint (see GrabSpec): a hidden shapeless kinematic
	// anchor driven to the target each step + a zero-length spring distance
	// joint from the grabbed body's local point to it. `body` remembers which
	// b3 body the joint was built against — when the group patches that index
	// to a new body (box3d destroyed the old one and our joint with it) the
	// stale handles are dropped and the constraint is rebuilt lazily.
	struct LiveGrab
	{
		GrabSpec spec;
		b3BodyId body = b3_nullBodyId;
		b3BodyId anchor = b3_nullBodyId;
		b3JointId joint = b3_nullJointId;
	};
	std::map<uint32_t, std::vector<LiveGrab>> grabNodes;

	void destroyWorld()
	{
		if ( B3_IS_NON_NULL( world ) )
		{
			b3DestroyWorld( world );
			world = b3_nullWorldId;
		}
		worldAnchorBody = b3_nullBodyId;
		worldStaticBodies.clear();
		for ( auto& entry : nodeJoints )
		{
			for ( NodeJoint& nj : entry.second )
			{
				nj.id = b3_nullJointId;
			}
		}
		for ( auto& entry : grabNodes )
		{
			for ( LiveGrab& g : entry.second )
			{
				g.body = b3_nullBodyId;
				g.anchor = b3_nullBodyId;
				g.joint = b3_nullJointId;
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
		worldDef.hitEventThreshold = settings.hitThreshold >= 0.0f ? settings.hitThreshold : 0.0f;
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
			worldStaticBodies.push_back( groundId );
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
				worldStaticBodies.push_back( wallId );
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
				worldStaticBodies.push_back( bodyId );
			}
		}

		// std::map iterates key-sorted, so respawn order is deterministic
		for ( auto& entry : groups )
		{
			Group& group = entry.second;
			createGroupBodies( world, entry.first, group );
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

		// Pivot mode overrides (see JointSpec): one shared anchor instead of
		// pivot-to-pivot. Mode 2 is the child-bone convention — the child bone's
		// pivot anchors both bodies, so a bone chained on both ends (thigh:
		// hip AND knee) articulates correctly with a single pivot per body.
		switch ( spec.pivotMode )
		{
			case 1: // body A's pivot anchors both
				worldPivotB = worldPivotA;
				break;
			case 2: // body B's pivot anchors both (falls back to A when B is the world)
				if ( !bodyBIsWorld )
				{
					worldPivotA = worldPivotB;
				}
				break;
			case 3: // explicit world-space anchor
				worldPivotA = b3Vec3{ spec.anchorX, spec.anchorY, spec.anchorZ };
				worldPivotB = worldPivotA;
				break;
			default:
				break;
		}

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
				target.p = b3Pos{ std::isfinite( def.px ) ? def.px : 0.0f, std::isfinite( def.py ) ? def.py : 0.0f,
								  std::isfinite( def.pz ) ? def.pz : 0.0f };
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

	// Tear down one grab's live constraint. The joint dies with the grabbed
	// body when a group patch recreates it, so validity is re-checked instead
	// of trusting the stored handles.
	void destroyGrabConstraint( LiveGrab& g )
	{
		if ( B3_IS_NON_NULL( g.joint ) && b3Joint_IsValid( g.joint ) )
		{
			b3DestroyJoint( g.joint, true ); // wake the released body so it falls/flings
		}
		if ( B3_IS_NON_NULL( g.anchor ) && b3Body_IsValid( g.anchor ) )
		{
			b3DestroyBody( g.anchor );
		}
		g.joint = b3_nullJointId;
		g.anchor = b3_nullBodyId;
		g.body = b3_nullBodyId;
	}

	// Create (or re-create) the live constraint of every registered grab.
	// Lazy: a grab whose group/body is not resolvable yet — or whose body was
	// recreated by a group patch, killing the old joint — retries on every
	// advance while its spec is present. Runs after the rebuild handling so a
	// fresh world re-acquires all grabs immediately.
	void updateGrabConstraints()
	{
		if ( grabNodes.empty() || !B3_IS_NON_NULL( world ) )
		{
			return;
		}
		for ( auto& entry : grabNodes )
		{
			for ( LiveGrab& g : entry.second )
			{
				b3BodyId body = b3_nullBodyId;
				auto git = groups.find( g.spec.groupKey );
				if ( git != groups.end() && g.spec.bodyIndex >= 0 &&
					 g.spec.bodyIndex < (int)git->second.bodies.size() )
				{
					body = git->second.bodies[g.spec.bodyIndex];
				}

				// Grabbed body gone or replaced: drop stale handles (the old
				// joint died with the old body; the anchor is still ours).
				if ( B3_IS_NON_NULL( g.body ) && ( !B3_IS_NON_NULL( body ) || !B3_ID_EQUALS( g.body, body ) ) )
				{
					destroyGrabConstraint( g );
				}
				if ( !B3_IS_NON_NULL( body ) || !b3Body_IsValid( body ) )
				{
					continue;
				}

				if ( !B3_IS_NON_NULL( g.joint ) )
				{
					// Anchor: shapeless kinematic spawned AT the target.
					b3BodyDef anchorDef = b3DefaultBodyDef();
					anchorDef.type = b3_kinematicBody;
					anchorDef.position = b3Pos{ g.spec.targetX, g.spec.targetY, g.spec.targetZ };
					g.anchor = b3CreateBody( world, &anchorDef );

					b3DistanceJointDef def = b3DefaultDistanceJointDef();
					def.base.bodyIdA = body;
					def.base.bodyIdB = g.anchor;
					def.base.localFrameA = b3Transform_identity;
					def.base.localFrameA.p = b3Vec3{ g.spec.localX, g.spec.localY, g.spec.localZ };
					def.base.localFrameB = b3Transform_identity;
					def.base.collideConnected = false;
					def.length = 0.0f;
					if ( g.spec.hertz > 0.0f )
					{
						def.enableSpring = true;
						def.hertz = g.spec.hertz;
						def.dampingRatio = g.spec.dampingRatio;
					}
					g.joint = b3CreateDistanceJoint( world, &def );
					g.body = body;
					b3Body_SetAwake( body, true );
				}
			}
		}
	}

	// Drive every grab anchor toward its target — called before each step,
	// like the kinematic retarget, so dragging imparts real velocity. Wakes
	// the grabbed body when the target actually moved (a sleeping body would
	// otherwise ignore the pull).
	void retargetGrabAnchors()
	{
		for ( auto& entry : grabNodes )
		{
			for ( LiveGrab& g : entry.second )
			{
				if ( !B3_IS_NON_NULL( g.anchor ) || !b3Body_IsValid( g.anchor ) )
				{
					continue;
				}
				b3WorldTransform target = b3WorldTransform_identity;
				target.p = b3Pos{ std::isfinite( g.spec.targetX ) ? g.spec.targetX : 0.0f,
								  std::isfinite( g.spec.targetY ) ? g.spec.targetY : 0.0f,
								  std::isfinite( g.spec.targetZ ) ? g.spec.targetZ : 0.0f };
				b3Pos p = b3Body_GetPosition( g.anchor );
				constexpr float kPosEps = 1e-6f;
				bool moved = fabsf( (float)( p.x - target.p.x ) ) > kPosEps ||
							 fabsf( (float)( p.y - target.p.y ) ) > kPosEps ||
							 fabsf( (float)( p.z - target.p.z ) ) > kPosEps;
				b3Body_SetTargetTransform( g.anchor, target, (float)kFixedTimeStep, false );
				if ( moved && B3_IS_NON_NULL( g.body ) && b3Body_IsValid( g.body ) )
				{
					b3Body_SetAwake( g.body, true );
				}
			}
		}
	}

	// Apply one force field to one dynamic body. `mass` is the body's mass.
	void applyFieldToBody( const ForceField& f, b3BodyId bodyId, float mass )
	{
		b3Pos c = b3Body_GetWorldCenter( bodyId );
		float fx = 0.0f, fy = 0.0f, fz = 0.0f; // acceleration (pre-mass) direction*magnitude

		if ( f.type == 2 ) // directional wind: constant, position-independent
		{
			float len = sqrtf( f.dx * f.dx + f.dy * f.dy + f.dz * f.dz );
			if ( len < 1e-8f )
				return;
			fx = f.strength * f.dx / len;
			fy = f.strength * f.dy / len;
			fz = f.strength * f.dz / len;
		}
		else
		{
			float dx = f.px - (float)c.x;
			float dy = f.py - (float)c.y;
			float dz = f.pz - (float)c.z;
			float dist = sqrtf( dx * dx + dy * dy + dz * dz );
			if ( dist < 1e-5f )
				return;
			if ( f.radius > 0.0f && dist > f.radius )
				return;

			float mag = f.strength;
			if ( f.falloff == 1 && f.radius > 0.0f ) // linear to the radius edge
			{
				mag *= ( 1.0f - dist / f.radius );
			}
			else if ( f.falloff == 2 ) // inverse-square
			{
				mag *= 1.0f / ( dist * dist );
			}

			if ( f.type == 3 ) // vortex: swirl around the axis through the point
			{
				float ax = f.dx, ay = f.dy, az = f.dz;
				float al = sqrtf( ax * ax + ay * ay + az * az );
				if ( al < 1e-8f )
					return;
				ax /= al;
				ay /= al;
				az /= al;
				// tangent = axis x radial
				float tx = ay * dz - az * dy;
				float ty = az * dx - ax * dz;
				float tz = ax * dy - ay * dx;
				float tl = sqrtf( tx * tx + ty * ty + tz * tz );
				if ( tl < 1e-6f )
					return;
				fx = mag * tx / tl;
				fy = mag * ty / tl;
				fz = mag * tz / tl;
			}
			else
			{
				// attractor pulls toward the point, repulsor pushes away
				float sign = f.type == 1 ? -1.0f : 1.0f;
				fx = sign * mag * dx / dist;
				fy = sign * mag * dy / dist;
				fz = sign * mag * dz / dist;
			}
		}

		// Treat the magnitude as acceleration by default (F = m*a) so every body
		// moves the same regardless of mass, like gravity; useMass applies it as
		// a raw force instead (heavier bodies respond less).
		float scale = f.useMass ? 1.0f : mass;
		b3Vec3 force = b3Vec3{ fx * scale, fy * scale, fz * scale };
		if ( std::isfinite( force.x ) && std::isfinite( force.y ) && std::isfinite( force.z ) )
		{
			b3Body_ApplyForceToCenter( bodyId, force, true );
		}
	}

	// Apply every registered force field once, before a step. box3d clears
	// accumulated forces after each Step, so this must run every step.
	void applyForceFields()
	{
		if ( forceNodes.empty() )
		{
			return;
		}

		for ( const auto& entry : forceNodes )
		{
			for ( const ForceField& f : entry.second )
			{
				auto applyToGroup = [&]( const Group& group ) {
					for ( size_t i = 0; i < group.bodies.size(); ++i )
					{
						b3BodyId bodyId = group.bodies[i];
						if ( !B3_IS_NON_NULL( bodyId ) || b3Body_GetType( bodyId ) != b3_dynamicBody )
						{
							continue;
						}
						float mass = b3Body_GetMass( bodyId );
						if ( mass <= 0.0f )
						{
							mass = 1.0f;
						}
						applyFieldToBody( f, bodyId, mass );
					}
				};

				if ( f.targetGroup != 0 )
				{
					auto it = groups.find( f.targetGroup );
					if ( it != groups.end() )
					{
						applyToGroup( it->second );
					}
				}
				else
				{
					for ( const auto& g : groups )
					{
						applyToGroup( g.second );
					}
				}
			}
		}
	}

	// Translate box3d's per-step event buffers into group/index events plus
	// per-body hit stats. Must run right after b3World_Step, while every id in
	// the buffers is still alive (bodies are only destroyed on client cooks,
	// never between the fixed steps of one advance).
	void captureContactEvents()
	{
		// Runaway guard: a degenerate pile can begin/end thousands of contacts
		// per step; anything past this is noise no one visualizes anyway.
		constexpr size_t kMaxEvents = 4096;

		b3ContactEvents ev = b3World_GetContactEvents( world );

		auto refFromShape = [&]( b3ShapeId shapeId, uint32_t& groupKey, int& index ) {
			groupKey = 0;
			index = -1;
			if ( b3Shape_IsValid( shapeId ) )
			{
				unpackBodyRef( b3Body_GetUserData( b3Shape_GetBody( shapeId ) ), groupKey, index );
			}
		};

		for ( int i = 0; i < ev.beginCount && events.size() < kMaxEvents; ++i )
		{
			const b3ContactBeginTouchEvent& b = ev.beginEvents[i];
			ContactEvent e;
			e.kind = 0;
			refFromShape( b.shapeIdA, e.groupA, e.indexA );
			refFromShape( b.shapeIdB, e.groupB, e.indexB );

			// Point and normal from the live manifold when it survived the step.
			if ( b3Contact_IsValid( b.contactId ) && b3Shape_IsValid( b.shapeIdA ) )
			{
				b3ContactData data = b3Contact_GetData( b.contactId );
				if ( data.manifolds != nullptr && data.manifoldCount > 0 && data.manifolds[0].pointCount > 0 )
				{
					const b3Manifold& man = data.manifolds[0];
					e.nx = man.normal.x;
					e.ny = man.normal.y;
					e.nz = man.normal.z;
					// anchorA is world-space relative to body A's center of mass.
					b3Pos c = b3Body_GetWorldCenter( b3Shape_GetBody( b.shapeIdA ) );
					e.px = (float)c.x + man.points[0].anchorA.x;
					e.py = (float)c.y + man.points[0].anchorA.y;
					e.pz = (float)c.z + man.points[0].anchorA.z;
				}
			}
			events.push_back( e );
		}

		for ( int i = 0; i < ev.endCount && events.size() < kMaxEvents; ++i )
		{
			const b3ContactEndTouchEvent& b = ev.endEvents[i];
			ContactEvent e;
			e.kind = 1;
			// End events can reference already-destroyed shapes; refFromShape
			// validates and falls back to the world ref.
			refFromShape( b.shapeIdA, e.groupA, e.indexA );
			refFromShape( b.shapeIdB, e.groupB, e.indexB );
			events.push_back( e );
		}

		for ( int i = 0; i < ev.hitCount && events.size() < kMaxEvents; ++i )
		{
			const b3ContactHitEvent& h = ev.hitEvents[i];
			ContactEvent e;
			e.kind = 2;
			refFromShape( h.shapeIdA, e.groupA, e.indexA );
			refFromShape( h.shapeIdB, e.groupB, e.indexB );
			e.px = (float)h.point.x;
			e.py = (float)h.point.y;
			e.pz = (float)h.point.z;
			e.nx = h.normal.x;
			e.ny = h.normal.y;
			e.nz = h.normal.z;
			e.speed = h.approachSpeed;
			events.push_back( e );

			if ( e.indexA >= 0 )
			{
				float& s = hitSpeeds[bodyRefKey( e.groupA, e.indexA )];
				s = e.speed > s ? e.speed : s;
			}
			if ( e.indexB >= 0 )
			{
				float& s = hitSpeeds[bodyRefKey( e.groupB, e.indexB )];
				s = e.speed > s ? e.speed : s;
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
		b3World_SetHitEventThreshold( m->world, settings.hitThreshold >= 0.0f ? settings.hitThreshold : 0.0f );
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

		// Something changed structurally (shape/size/type/count/hull). Mesh
		// shapes pool their b3MeshData per group without per-body tracking,
		// so any mesh involvement falls back to the full recreate below;
		// everything else gets patched PER INDEX: bodies whose def did not
		// change keep their live body — and its simulated state — untouched.
		// This is what lets an emitter-style Spawn SOP grow or shrink its
		// point count without resetting the bodies already simulating. Body
		// identity is the point index: keep upstream point order stable and
		// append new points at the end.
		bool anyMesh = false;
		for ( const SpawnBody& d : group.defs )
		{
			anyMesh = anyMesh || d.shape == 4;
		}
		for ( const SpawnBody& d : defs )
		{
			anyMesh = anyMesh || d.shape == 4;
		}

		if ( !anyMesh )
		{
			const size_t oldCount = group.bodies.size() < group.defs.size() ? group.bodies.size() : group.defs.size();
			const size_t newCount = defs.size();
			const size_t common = oldCount < newCount ? oldCount : newCount;

			std::vector<b3BodyId> newBodies( newCount, b3_nullBodyId );
			bool structural = group.bodies.size() != newCount;

			for ( size_t i = 0; i < common; ++i )
			{
				if ( sameShapeAndType( group.defs[i], defs[i] ) && B3_IS_NON_NULL( group.bodies[i] ) )
				{
					// Unchanged: the live body carries over as-is.
					newBodies[i] = group.bodies[i];
					if ( !sameMaterial( group.defs[i], defs[i] ) )
					{
						applyMaterialToBody( newBodies[i], defs[i] );
					}
					if ( defs[i].type != 2 && !samePose( group.defs[i], defs[i] ) )
					{
						setBodyTransformFromDef( newBodies[i], defs[i] );
					}
					if ( !sameJointPivot( group.defs[i], defs[i] ) )
					{
						m->jointsDirty = true;
					}
					continue;
				}

				// This index changed: recreate it, preserving the dynamic
				// state when the spawn pose is unchanged (finite state only).
				structural = true;
				bool keep = false;
				b3Pos kp;
				b3Quat kq;
				b3Vec3 kv, kw;
				if ( defs[i].type == 2 && group.defs[i].type == 2 && samePose( group.defs[i], defs[i] ) &&
					 B3_IS_NON_NULL( group.bodies[i] ) )
				{
					kp = b3Body_GetPosition( group.bodies[i] );
					kq = b3Body_GetRotation( group.bodies[i] );
					kv = b3Body_GetLinearVelocity( group.bodies[i] );
					kw = b3Body_GetAngularVelocity( group.bodies[i] );
					keep = std::isfinite( (float)kp.x ) && std::isfinite( (float)kp.y ) && std::isfinite( (float)kp.z ) &&
						   std::isfinite( kq.v.x ) && std::isfinite( kq.v.y ) && std::isfinite( kq.v.z ) &&
						   std::isfinite( kq.s ) && std::isfinite( kv.x ) && std::isfinite( kv.y ) &&
						   std::isfinite( kv.z ) && std::isfinite( kw.x ) && std::isfinite( kw.y ) &&
						   std::isfinite( kw.z );
				}
				if ( B3_IS_NON_NULL( group.bodies[i] ) )
				{
					b3DestroyBody( group.bodies[i] );
				}
				std::vector<b3BodyId> made;
				createBodyFromDef( m->world, defs[i], made, group.meshes );
				newBodies[i] = made.empty() ? b3_nullBodyId : made[0];
				if ( B3_IS_NON_NULL( newBodies[i] ) )
				{
					b3Body_SetUserData( newBodies[i], packBodyRef( groupKey, (int)i ) );
				}
				if ( keep && B3_IS_NON_NULL( newBodies[i] ) )
				{
					b3Body_SetTransform( newBodies[i], kp, kq );
					b3Body_SetLinearVelocity( newBodies[i], kv );
					b3Body_SetAngularVelocity( newBodies[i], kw );
				}
			}

			// Shrunk: drop the tail. Grown: spawn the new tail.
			for ( size_t i = common; i < group.bodies.size(); ++i )
			{
				if ( B3_IS_NON_NULL( group.bodies[i] ) )
				{
					b3DestroyBody( group.bodies[i] );
					structural = true;
				}
			}
			for ( size_t i = common; i < newCount; ++i )
			{
				std::vector<b3BodyId> made;
				createBodyFromDef( m->world, defs[i], made, group.meshes );
				newBodies[i] = made.empty() ? b3_nullBodyId : made[0];
				if ( B3_IS_NON_NULL( newBodies[i] ) )
				{
					b3Body_SetUserData( newBodies[i], packBodyRef( groupKey, (int)i ) );
				}
				structural = true;
			}

			group.bodies = std::move( newBodies );
			group.defs = std::move( defs );
			if ( structural )
			{
				// box3d killed any joints attached to destroyed bodies, and
				// index-series joints may now resolve further (or less far).
				m->jointsDirty = true;
			}
			return;
		}

		// Mesh involved: full recreate (per-group mesh blobs die together).
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

					// Never carry a corrupted state into the fresh body: if the
					// old one blew up (NaN pose/velocity from a degenerate
					// collider), respawning at the spawn pose is the recovery.
					bool finite = std::isfinite( (float)preserved[i].p.x ) && std::isfinite( (float)preserved[i].p.y ) &&
								  std::isfinite( (float)preserved[i].p.z ) && std::isfinite( preserved[i].q.v.x ) &&
								  std::isfinite( preserved[i].q.v.y ) && std::isfinite( preserved[i].q.v.z ) &&
								  std::isfinite( preserved[i].q.s ) && std::isfinite( preserved[i].v.x ) &&
								  std::isfinite( preserved[i].v.y ) && std::isfinite( preserved[i].v.z ) &&
								  std::isfinite( preserved[i].w.x ) && std::isfinite( preserved[i].w.y ) &&
								  std::isfinite( preserved[i].w.z );
					if ( !finite )
					{
						preserved[i].valid = false;
					}
				}
			}
		}

		destroyGroupBodies( group );
		group.defs = std::move( defs );
		createGroupBodies( m->world, groupKey, group );
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
		createGroupBodies( m->world, groupKey, group );
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
	if ( it == m->groups.end() )
	{
		return;
	}
	// Clients call this every cook, so it doubles as the group heartbeat — refresh
	// it even if no path was supplied, or a null opPath would let the group be
	// dropped as stale within a few advances.
	it->second.lastTouch = m->advanceCounter;
	if ( path != nullptr && it->second.path != path )
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

	// Clients call this every cook, so it doubles as the joints heartbeat.
	m->jointTouch[ownerKey] = m->advanceCounter;

	if ( it != m->nodeJoints.end() )
	{
		std::vector<Impl::NodeJoint>& current = it->second;
		if ( current.size() == specs.size() )
		{
			bool identical = true;	// nothing changed at all
			bool springOnly = true; // only spring params (hertz/damping/length) changed
			bool liveUpdatable = B3_IS_NON_NULL( m->world );
			for ( size_t i = 0; i < specs.size(); ++i )
			{
				if ( !sameJointSpec( current[i].spec, specs[i] ) )
				{
					identical = false;
				}
				if ( !sameJointStructure( current[i].spec, specs[i] ) || !jointTypeSupportsLiveSpring( specs[i].type ) )
				{
					springOnly = false;
				}
				if ( !( B3_IS_NON_NULL( current[i].id ) && b3Joint_IsValid( current[i].id ) ) )
				{
					liveUpdatable = false;
				}
			}

			if ( identical )
			{
				return;
			}

			// Only the spring stiffness/damping/length changed on already-built
			// joints: update them on the live joints instead of destroying and
			// recreating everything. This is what keeps dragging a Cloth's
			// Stiffness/Damping (or the Joint CHOP spring sliders) from
			// rebuilding thousands of joints every cook and stalling the sim.
			if ( springOnly && liveUpdatable )
			{
				for ( size_t i = 0; i < specs.size(); ++i )
				{
					applyLiveSpringParams( current[i].id, specs[i] );
					current[i].spec = specs[i];
				}
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
	m->jointTouch.erase( ownerKey );
}

void SolverCore::setForceNodeList( uint32_t ownerKey, const std::vector<ForceField>& fields )
{
	Impl* m = myImpl;
	if ( fields.empty() )
	{
		removeForceNode( ownerKey );
		return;
	}
	// Clients call this every cook, so it doubles as the force heartbeat.
	m->forceTouch[ownerKey] = m->advanceCounter;
	m->forceNodes[ownerKey] = fields; // force fields read fresh each step; no diffing needed
}

void SolverCore::removeForceNode( uint32_t ownerKey )
{
	Impl* m = myImpl;
	m->forceNodes.erase( ownerKey );
	m->forceTouch.erase( ownerKey );
}

void SolverCore::setGrabList( uint32_t ownerKey, const std::vector<GrabSpec>& grabs )
{
	Impl* m = myImpl;
	if ( grabs.empty() )
	{
		removeGrabNode( ownerKey );
		return;
	}
	// Clients call this every cook, so it doubles as the grab heartbeat.
	m->grabTouch[ownerKey] = m->advanceCounter;

	std::vector<Impl::LiveGrab>& live = m->grabNodes[ownerKey];
	const size_t common = live.size() < grabs.size() ? live.size() : grabs.size();
	for ( size_t i = 0; i < common; ++i )
	{
		Impl::LiveGrab& g = live[i];
		const GrabSpec& spec = grabs[i];
		bool sameHold = g.spec.groupKey == spec.groupKey && g.spec.bodyIndex == spec.bodyIndex &&
						g.spec.localX == spec.localX && g.spec.localY == spec.localY &&
						g.spec.localZ == spec.localZ && ( g.spec.hertz > 0.0f ) == ( spec.hertz > 0.0f );
		if ( !sameHold )
		{
			// Grabbed a different point/body (or crossed rigid<->spring):
			// release and let advance() build the new constraint.
			m->destroyGrabConstraint( g );
		}
		else if ( B3_IS_NON_NULL( g.joint ) && b3Joint_IsValid( g.joint ) && spec.hertz > 0.0f &&
				  ( g.spec.hertz != spec.hertz || g.spec.dampingRatio != spec.dampingRatio ) )
		{
			// Same hold, different feel: retune the spring live.
			b3DistanceJoint_SetSpringHertz( g.joint, spec.hertz );
			b3DistanceJoint_SetSpringDampingRatio( g.joint, spec.dampingRatio );
		}
		g.spec = spec; // target (and spring numbers) always take the new values
	}
	for ( size_t i = grabs.size(); i < live.size(); ++i )
	{
		m->destroyGrabConstraint( live[i] ); // released holds
	}
	live.resize( grabs.size() );
	for ( size_t i = common; i < grabs.size(); ++i )
	{
		live[i].spec = grabs[i]; // new holds; constraints built lazily in advance()
	}
}

void SolverCore::removeGrabNode( uint32_t ownerKey )
{
	Impl* m = myImpl;
	auto it = m->grabNodes.find( ownerKey );
	if ( it != m->grabNodes.end() )
	{
		for ( Impl::LiveGrab& g : it->second )
		{
			m->destroyGrabConstraint( g );
		}
		m->grabNodes.erase( it );
	}
	m->grabTouch.erase( ownerKey );
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

void SolverCore::getGroupWireframe( uint32_t groupKey, std::vector<float>& segments, std::vector<float>& colors ) const
{
	const Impl* m = myImpl;
	if ( !B3_IS_NON_NULL( m->world ) )
	{
		return;
	}
	auto it = m->groups.find( groupKey );
	if ( it != m->groups.end() )
	{
		debugAppendGroup( segments, colors, it->second );
	}
}

void SolverCore::getDebugWireframe( bool bodies, bool worldStatics, bool joints, std::vector<float>& segments,
									std::vector<float>& colors ) const
{
	const Impl* m = myImpl;
	if ( !B3_IS_NON_NULL( m->world ) )
	{
		return;
	}

	constexpr float kWorldColor[3] = { 0.35f, 0.35f, 0.38f };
	constexpr float kJointColor[3] = { 1.0f, 0.9f, 0.15f };

	if ( bodies )
	{
		for ( const auto& entry : m->groups )
		{
			debugAppendGroup( segments, colors, entry.second );
		}
	}

	if ( worldStatics )
	{
		for ( b3BodyId bodyId : m->worldStaticBodies )
		{
			debugAppendBody( segments, colors, kWorldColor, bodyId );
		}
	}

	if ( joints )
	{
		for ( const auto& entry : m->nodeJoints )
		{
			for ( const Impl::NodeJoint& nj : entry.second )
			{
				if ( !B3_IS_NON_NULL( nj.id ) || !b3Joint_IsValid( nj.id ) )
				{
					continue;
				}
				b3Transform xfA = toLocalTransform( nj.bodyA );
				b3Transform xfB = toLocalTransform( nj.bodyB );
				b3Vec3 a = b3Add( b3RotateVector( xfA.q, nj.localA.p ), xfA.p );
				b3Vec3 b = b3Add( b3RotateVector( xfB.q, nj.localB.p ), xfB.p );
				debugPushSegment( segments, colors, kJointColor, a, b );

				// Small cross at each anchor so pinned (zero-length) joints
				// are still visible.
				constexpr float kTick = 0.05f;
				const b3Vec3 anchors[2] = { a, b };
				for ( const b3Vec3& p : anchors )
				{
					debugPushSegment( segments, colors, kJointColor, b3Vec3{ p.x - kTick, p.y, p.z },
									  b3Vec3{ p.x + kTick, p.y, p.z } );
					debugPushSegment( segments, colors, kJointColor, b3Vec3{ p.x, p.y - kTick, p.z },
									  b3Vec3{ p.x, p.y + kTick, p.z } );
					debugPushSegment( segments, colors, kJointColor, b3Vec3{ p.x, p.y, p.z - kTick },
									  b3Vec3{ p.x, p.y, p.z + kTick } );
				}
			}
		}
	}
}

void SolverCore::requestRebuild()
{
	myImpl->dirty = true;
}

void SolverCore::advance( double dtSeconds, bool simulate )
{
	Impl* m = myImpl;
	++m->advanceCounter;

	// Drop groups/joints whose owner node stopped cooking (bypassed, cook
	// flag off, deleted COMP...). Clients cook every frame and touch their
	// registrations each cook, so a few advances of silence means the node
	// is out of the network's cook loop and its physics must go with it.
	// Un-bypassing re-registers everything on the node's next cook.
	constexpr uint64_t kStaleAdvances = 4;
	if ( m->advanceCounter > kStaleAdvances )
	{
		std::vector<uint32_t> stale;
		for ( const auto& entry : m->groups )
		{
			if ( m->advanceCounter - entry.second.lastTouch > kStaleAdvances )
			{
				stale.push_back( entry.first );
			}
		}
		for ( uint32_t key : stale )
		{
			removeGroup( key );
		}

		stale.clear();
		for ( const auto& entry : m->jointTouch )
		{
			if ( m->advanceCounter - entry.second > kStaleAdvances )
			{
				stale.push_back( entry.first );
			}
		}
		for ( uint32_t key : stale )
		{
			removeJointNode( key );
		}

		stale.clear();
		for ( const auto& entry : m->forceTouch )
		{
			if ( m->advanceCounter - entry.second > kStaleAdvances )
			{
				stale.push_back( entry.first );
			}
		}
		for ( uint32_t key : stale )
		{
			removeForceNode( key );
		}

		stale.clear();
		for ( const auto& entry : m->grabTouch )
		{
			if ( m->advanceCounter - entry.second > kStaleAdvances )
			{
				stale.push_back( entry.first );
			}
		}
		for ( uint32_t key : stale )
		{
			removeGrabNode( key );
		}
	}

	if ( m->dirty )
	{
		m->rebuild();
	}

	if ( m->jointsDirty )
	{
		m->syncJoints();
	}

	// New advance = new event window: events already reported must not repeat,
	// even when paused or when no fixed step fits into this cook's delta.
	m->events.clear();
	m->hitSpeeds.clear();

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

	// (Re)build any pending grab constraints now that the world/groups are
	// settled for this advance — before stepping so a fresh grab bites this
	// same frame.
	m->updateGrabConstraints();

	int steps = 0;
	while ( m->accumulator >= kFixedTimeStep && steps < maxSteps )
	{
		m->retargetKinematicBodies();
		m->retargetGrabAnchors();
		m->applyForceFields();
		b3World_Step( m->world, (float)kFixedTimeStep, m->settings.subSteps );
		m->captureContactEvents();
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

int SolverCore::contactEventCount() const
{
	return (int)myImpl->events.size();
}

int SolverCore::getContactEvents( ContactEvent* out, int capacity ) const
{
	const Impl* m = myImpl;
	int count = (int)m->events.size() < capacity ? (int)m->events.size() : capacity;
	for ( int i = 0; i < count; ++i )
	{
		out[i] = m->events[i];
	}
	return count;
}

int SolverCore::getGroupContactStates( uint32_t groupKey, BodyContactState* out, int capacity ) const
{
	const Impl* m = myImpl;
	auto it = m->groups.find( groupKey );
	if ( it == m->groups.end() )
	{
		return 0;
	}

	const Group& group = it->second;

	// Not built yet (world rebuild pending): report zeros, same fallback
	// convention as getGroupStates.
	if ( group.bodies.empty() )
	{
		int count = (int)group.defs.size() < capacity ? (int)group.defs.size() : capacity;
		for ( int i = 0; i < count; ++i )
		{
			out[i] = BodyContactState{};
		}
		return count;
	}

	int count = (int)group.bodies.size() < capacity ? (int)group.bodies.size() : capacity;
	std::vector<b3ContactData> contacts;
	for ( int i = 0; i < count; ++i )
	{
		BodyContactState s;
		b3BodyId bodyId = group.bodies[i];
		if ( B3_IS_NON_NULL( bodyId ) )
		{
			int cap = b3Body_GetContactCapacity( bodyId );
			if ( cap > 0 )
			{
				contacts.resize( cap );
				int n = b3Body_GetContactData( bodyId, contacts.data(), cap );
				for ( int c = 0; c < n; ++c )
				{
					// A contact is "touching" when any manifold has points;
					// impulses sum over every point (speculative points with a
					// zero impulse contribute nothing).
					bool touching = false;
					for ( int mi = 0; mi < contacts[c].manifoldCount; ++mi )
					{
						const b3Manifold& man = contacts[c].manifolds[mi];
						if ( man.pointCount > 0 )
						{
							touching = true;
						}
						for ( int p = 0; p < man.pointCount; ++p )
						{
							s.impulse += man.points[p].totalNormalImpulse;
						}
					}
					if ( touching )
					{
						s.touching += 1.0f;
					}
				}
			}

			auto hit = m->hitSpeeds.find( bodyRefKey( groupKey, i ) );
			if ( hit != m->hitSpeeds.end() )
			{
				s.hitSpeed = hit->second;
			}
		}
		out[i] = s;
	}
	return count;
}

bool SolverCore::getGroupPathByKey( uint32_t groupKey, std::string& outPath ) const
{
	const Impl* m = myImpl;
	auto it = m->groups.find( groupKey );
	if ( it == m->groups.end() )
	{
		return false;
	}
	outPath = it->second.path;
	return true;
}

bool SolverCore::findGroupKeyByPath( const char* ref, uint32_t& outKey ) const
{
	const Impl* m = myImpl;
	if ( ref == nullptr || ref[0] == '\0' )
	{
		return false;
	}
	for ( const auto& entry : m->groups )
	{
		if ( pathMatches( entry.second.path, ref ) )
		{
			outKey = entry.first;
			return true;
		}
	}
	return false;
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

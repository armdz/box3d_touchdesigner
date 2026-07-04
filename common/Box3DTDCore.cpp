#include "Box3DTDCore.h"

#include "box3d/box3d.h"

#include <cmath>
#include <map>

namespace tdb3
{

namespace
{

constexpr double kFixedTimeStep = 1.0 / 60.0;

// Cap the number of fixed steps per advance so a long stall (file dialog,
// heavy cook) doesn't make the solver spiral trying to catch up.
constexpr int kMaxStepsPerAdvance = 4;

struct Group
{
	std::vector<SpawnBody> defs;
	std::vector<b3BodyId> bodies;
};

void createBodyFromDef( b3WorldId world, const SpawnBody& def, std::vector<b3BodyId>& outBodies )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = def.type == 0 ? b3_staticBody : ( def.type == 1 ? b3_kinematicBody : b3_dynamicBody );
	bodyDef.position = b3Pos{ def.px, def.py, def.pz };

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
				b3HullData* hull = b3CreateHull( (const b3Vec3*)def.hullPoints.data(), pointCount, 64 );
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
		default: // box
		{
			b3BoxHull hull = b3MakeBoxHull( 0.5f * sizeX, 0.5f * sizeY, 0.5f * sizeZ );
			b3CreateHullShape( bodyId, &shapeDef, &hull.base );
			break;
		}
	}

	outBodies.push_back( bodyId );
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
		for ( auto& entry : groups )
		{
			entry.second.bodies.clear();
		}
		// The mesh can only die after the world (its mesh shape) is gone
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
		worldDef.gravity = b3Vec3{ settings.gravityX, settings.gravityY, settings.gravityZ };
		worldDef.workerCount = 1;
		world = b3CreateWorld( &worldDef );

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

		if ( hasMesh && !meshIndices.empty() )
		{
			b3MeshDef meshDef = {};
			meshDef.vertices = meshVertices.data();
			meshDef.vertexCount = (int)meshVertices.size();
			meshDef.indices = meshIndices.data();
			meshDef.triangleCount = (int)meshIndices.size() / 3;
			meshDef.identifyEdges = true;

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
			group.bodies.reserve( group.defs.size() );
			for ( const SpawnBody& def : group.defs )
			{
				createBodyFromDef( world, def, group.bodies );
			}
		}

		accumulator = 0.0;
		stepCount = 0;
		dirty = false;
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
	if ( settings.ground != m->settings.ground || settings.groundSize != m->settings.groundSize )
	{
		m->dirty = true;
	}
	m->settings = settings;

	if ( B3_IS_NON_NULL( m->world ) && !m->dirty )
	{
		b3World_SetGravity( m->world, b3Vec3{ settings.gravityX, settings.gravityY, settings.gravityZ } );
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
		m->groups.erase( it );
		m->dirty = true;
	}
}

bool SolverCore::hasGroup( uint32_t groupKey ) const
{
	return myImpl->groups.find( groupKey ) != myImpl->groups.end();
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

	if ( !simulate )
	{
		return;
	}

	m->accumulator += dtSeconds;

	int steps = 0;
	while ( m->accumulator >= kFixedTimeStep && steps < kMaxStepsPerAdvance )
	{
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

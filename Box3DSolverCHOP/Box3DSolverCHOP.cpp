#include "Box3DSolverCHOP.h"

#include <cmath>
#include <cstring>

using namespace tdb3;

namespace
{

constexpr double kFixedTimeStep = 1.0 / 60.0;

// Solver-only parameter names
constexpr char SimulateName[] = "Simulate";
constexpr char ResetName[] = "Reset";
constexpr char GravityName[] = "Gravity";
constexpr char SubstepsName[] = "Substeps";
constexpr char GroundName[] = "Ground";
constexpr char GroundsizeName[] = "Groundsize";
constexpr char CollisionsopName[] = "Collisionsop";
constexpr char BoxcountName[] = "Boxcount";
constexpr char SpawnheightName[] = "Spawnheight";

WorldSettings readWorldSettings( const OP_Inputs* inputs )
{
	WorldSettings s;

	double gx, gy, gz;
	inputs->getParDouble3( GravityName, gx, gy, gz );
	s.gravityX = (float)gx;
	s.gravityY = (float)gy;
	s.gravityZ = (float)gz;

	s.ground = inputs->getParInt( GroundName ) != 0;
	s.groundSize = (float)inputs->getParDouble( GroundsizeName );
	s.subSteps = inputs->getParInt( SubstepsName );
	return s;
}

// Gather triangles from a SOP, fan-triangulating polygons
void extractTriangleSoup( const OP_SOPInput* sop, std::vector<float>& vertices, std::vector<int32_t>& indices )
{
	int pointCount = sop->getNumPoints();
	const Position* points = sop->getPointPositions();

	vertices.reserve( pointCount * 3 );
	for ( int i = 0; i < pointCount; ++i )
	{
		vertices.push_back( points[i].x );
		vertices.push_back( points[i].y );
		vertices.push_back( points[i].z );
	}

	int primCount = sop->getNumPrimitives();
	indices.reserve( primCount * 3 );
	for ( int i = 0; i < primCount; ++i )
	{
		const SOP_PrimitiveInfo& prim = sop->getPrimitive( i );
		if ( prim.type != PrimitiveType::Polygon || prim.numVertices < 3 )
		{
			continue;
		}

		for ( int v = 1; v < prim.numVertices - 1; ++v )
		{
			indices.push_back( prim.pointIndices[0] );
			indices.push_back( prim.pointIndices[v] );
			indices.push_back( prim.pointIndices[v + 1] );
		}
	}
}

} // namespace

extern "C"
{

DLLEXPORT
void FillCHOPPluginInfo( CHOP_PluginInfo* info )
{
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString( "Box3dsolver" );
	customInfo.opLabel->setString( "Box3D Solver" );
	// Max 3 chars, letters only (no digits allowed by TD)
	customInfo.opIcon->setString( "BOX" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase* CreateCHOPInstance( const OP_NodeInfo* info )
{
	return new Box3DSolverCHOP( info );
}

DLLEXPORT
void DestroyCHOPInstance( CHOP_CPlusPlusBase* instance )
{
	delete (Box3DSolverCHOP*)instance;
}

} // extern "C"

Box3DSolverCHOP::Box3DSolverCHOP( const OP_NodeInfo* info ) : myOpId( info->opId )
{
}

Box3DSolverCHOP::~Box3DSolverCHOP()
{
	Registry::destroy( myOpId );
}

void Box3DSolverCHOP::getGeneralInfo( CHOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// The simulation must advance every frame while its output is in use.
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = false;
}

bool Box3DSolverCHOP::getOutputInfo( CHOP_OutputInfo* info, const OP_Inputs* inputs, void* )
{
	int sampleCount = inputs->getParInt( BoxcountName );

	const OP_SOPInput* sop = inputs->getParSOP( SpawnsopParName );
	if ( sop != nullptr )
	{
		sampleCount = sop->getNumPoints();
	}
	else
	{
		SolverCore* core = Registry::find( myOpId );
		if ( core != nullptr && core->groupCount() > ( core->hasGroup( myOpId ) ? 1 : 0 ) )
		{
			// Bodies nodes drive this world; the demo grid is off
			sampleCount = 0;
		}
	}

	info->numChannels = kNumOutputChannels;
	info->numSamples = sampleCount > 0 ? sampleCount : 1;
	info->startIndex = 0;
	return true;
}

void Box3DSolverCHOP::getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* )
{
	name->setString( kOutputChannelNames[index] );
}

std::vector<SpawnBody> Box3DSolverCHOP::buildDemoGridDefs( const OP_Inputs* inputs,
														   const SpawnDefaults& defaults ) const
{
	// Deterministic grid above the ground. A tiny index-based jitter breaks up
	// perfectly aligned stacks; no rand() so Reset always reproduces the same
	// simulation.
	int boxCount = inputs->getParInt( BoxcountName );
	float spawnHeight = (float)inputs->getParDouble( SpawnheightName );
	float spacing = 1.5f * ( defaults.sizeX > defaults.sizeY ? defaults.sizeX : defaults.sizeY );

	std::vector<SpawnBody> defs;
	defs.reserve( boxCount );
	for ( int i = 0; i < boxCount; ++i )
	{
		int col = i % 4;
		int row = ( i / 4 ) % 4;
		int layer = i / 16;

		float jitter = 0.05f * defaults.sizeX * sinf( 12.9898f * (float)( i + 1 ) );

		SpawnBody d;
		d.px = ( col - 1.5f ) * spacing + jitter;
		d.py = spawnHeight + layer * spacing;
		d.pz = ( row - 1.5f ) * spacing - jitter;
		d.shape = defaults.shape;
		d.sizeX = defaults.sizeX;
		d.sizeY = defaults.sizeY;
		d.sizeZ = defaults.sizeZ;
		d.density = defaults.density;
		d.friction = defaults.friction;

		defs.push_back( d );
	}

	return defs;
}

void Box3DSolverCHOP::execute( CHOP_Output* output, const OP_Inputs* inputs, void* )
{
	SolverCore* core = Registry::getOrCreate( myOpId );

	core->setWorldSettings( readWorldSettings( inputs ) );

	const OP_SOPInput* spawnSop = inputs->getParSOP( SpawnsopParName );
	const OP_SOPInput* collisionSop = inputs->getParSOP( CollisionsopName );
	bool hasSop = spawnSop != nullptr;
	bool hasCollision = collisionSop != nullptr;

	// The demo grid only runs when this solver is alone: no Spawn SOP and no
	// Bodies nodes contributing groups.
	bool hasExternalGroups = core->groupCount() > ( core->hasGroup( myOpId ) ? 1 : 0 );
	bool demoMode = !hasSop && !hasExternalGroups;

	// The demo-grid parameters only apply in demo mode
	inputs->enablePar( BoxcountName, demoMode );
	inputs->enablePar( SpawnheightName, demoMode );
	inputs->enablePar( ResetoninputParName, hasSop || hasCollision );

	bool resetOnInput = inputs->getParInt( ResetoninputParName ) != 0;
	SpawnDefaults defaults = readSpawnDefaults( inputs );

	// ---- own spawn group ----
	uint32_t sopId = hasSop ? spawnSop->opId : 0;
	int64_t sopCooks = hasSop ? spawnSop->totalCooks : -1;
	int boxCount = inputs->getParInt( BoxcountName );
	float spawnHeight = (float)inputs->getParDouble( SpawnheightName );

	bool groupChanged = !myGroupRegistered || defaults != myLastDefaults || demoMode != myLastDemoMode ||
						sopId != mySopId || ( hasSop && resetOnInput && sopCooks != mySopCooks ) ||
						( demoMode && ( boxCount != myLastBoxCount || spawnHeight != myLastSpawnHeight ) );

	if ( groupChanged )
	{
		if ( hasSop )
		{
			core->setGroup( myOpId, parseSpawnSop( spawnSop, defaults ) );
		}
		else if ( demoMode )
		{
			core->setGroup( myOpId, buildDemoGridDefs( inputs, defaults ) );
		}
		else
		{
			core->setGroup( myOpId, {} );
		}

		myGroupRegistered = true;
		myLastDefaults = defaults;
		myLastDemoMode = demoMode;
		mySopId = sopId;
		mySopCooks = sopCooks;
		myLastBoxCount = boxCount;
		myLastSpawnHeight = spawnHeight;
	}

	// ---- collision mesh ----
	uint32_t collisionId = hasCollision ? collisionSop->opId : 0;
	int64_t collisionCooks = hasCollision ? collisionSop->totalCooks : -1;
	bool collisionChanged = collisionId != myCollisionSopId ||
							( hasCollision && resetOnInput && collisionCooks != myCollisionSopCooks );

	if ( collisionChanged )
	{
		if ( hasCollision )
		{
			std::vector<float> vertices;
			std::vector<int32_t> indices;
			extractTriangleSoup( collisionSop, vertices, indices );
			if ( !indices.empty() )
			{
				core->setCollisionMesh( vertices.data(), (int)vertices.size() / 3, indices.data(), (int)indices.size() );
			}
			else
			{
				core->clearCollisionMesh();
			}
		}
		else
		{
			core->clearCollisionMesh();
		}

		myCollisionSopId = collisionId;
		myCollisionSopCooks = collisionCooks;
	}

	if ( myResetPending )
	{
		core->requestRebuild();
		myResetPending = false;
	}

	// ---- step ----
	bool simulate = inputs->getParInt( SimulateName ) != 0;

	const OP_TimeInfo* timeInfo = inputs->getTimeInfo();
	// deltaMS is 0 on the first cook after load
	double deltaSeconds = timeInfo != nullptr ? timeInfo->deltaMS / 1000.0 : kFixedTimeStep;

	core->advance( deltaSeconds, simulate );

	// ---- output own group ----
	writeTransformChannels( output, core, myOpId );
}

int32_t Box3DSolverCHOP::getNumInfoCHOPChans( void* )
{
	return 3;
}

void Box3DSolverCHOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	SolverCore* core = Registry::find( myOpId );

	if ( index == 0 )
	{
		chan->name->setString( "awake_bodies" );
		chan->value = core != nullptr ? (float)core->awakeBodyCount() : 0.0f;
	}
	else if ( index == 1 )
	{
		chan->name->setString( "body_count" );
		chan->value = core != nullptr ? (float)core->totalBodyCount() : 0.0f;
	}
	else
	{
		chan->name->setString( "step_count" );
		chan->value = core != nullptr ? (float)core->stepCount() : 0.0f;
	}
}

void Box3DSolverCHOP::setupParameters( OP_ParameterManager* manager, void* )
{
	// ---- Solver page ----
	{
		OP_NumericParameter p;
		p.name = SimulateName;
		p.label = "Simulate";
		p.page = "Solver";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = ResetName;
		p.label = "Reset";
		p.page = "Solver";
		manager->appendPulse( p );
	}

	{
		OP_NumericParameter p;
		p.name = GravityName;
		p.label = "Gravity";
		p.page = "Solver";
		p.defaultValues[0] = 0.0;
		p.defaultValues[1] = -10.0;
		p.defaultValues[2] = 0.0;
		for ( int i = 0; i < 3; ++i )
		{
			p.minSliders[i] = -20.0;
			p.maxSliders[i] = 20.0;
		}
		manager->appendXYZ( p );
	}

	{
		OP_NumericParameter p;
		p.name = SubstepsName;
		p.label = "Sub Steps";
		p.page = "Solver";
		p.defaultValues[0] = 4;
		p.minValues[0] = 1;
		p.maxValues[0] = 16;
		p.minSliders[0] = 1;
		p.maxSliders[0] = 8;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		manager->appendInt( p );
	}

	{
		OP_NumericParameter p;
		p.name = GroundName;
		p.label = "Ground Plane";
		p.page = "Solver";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = GroundsizeName;
		p.label = "Ground Size";
		p.page = "Solver";
		p.defaultValues[0] = 20.0;
		p.minValues[0] = 1.0;
		p.minSliders[0] = 1.0;
		p.maxSliders[0] = 100.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_StringParameter p;
		p.name = CollisionsopName;
		p.label = "Collision SOP";
		p.page = "Solver";
		manager->appendSOP( p );
	}

	// ---- Bodies page (own spawn group + demo grid) ----
	appendBodyParameters( manager, "Bodies" );

	{
		OP_NumericParameter p;
		p.name = BoxcountName;
		p.label = "Demo Box Count";
		p.page = "Bodies";
		p.defaultValues[0] = 24;
		p.minValues[0] = 1;
		p.maxValues[0] = 4096;
		p.minSliders[0] = 1;
		p.maxSliders[0] = 256;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		manager->appendInt( p );
	}

	{
		OP_NumericParameter p;
		p.name = SpawnheightName;
		p.label = "Demo Spawn Height";
		p.page = "Bodies";
		p.defaultValues[0] = 5.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 20.0;
		manager->appendFloat( p );
	}
}

void Box3DSolverCHOP::pulsePressed( const char* name, void* )
{
	if ( std::strcmp( name, ResetName ) == 0 )
	{
		myResetPending = true;
	}
}

#include "Box3DBodiesCHOP.h"

#include <cmath>
#include <cstring>

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char ResetName[] = "Reset";
constexpr char MaterialpresetName[] = "Materialpreset";
constexpr int kInstancesNumOutputChannels = 9;
constexpr const char* kInstancesOutputChannelNames[kInstancesNumOutputChannels] = { "tx", "ty", "tz", "rx", "ry",
																   "rz", "sx", "sy", "sz" };

void applyMaterialPreset( int preset, SpawnDefaults& defaults )
{
	switch ( preset )
	{
		case 1: // Soft
			defaults.density = 1.0f;
			defaults.friction = 0.85f;
			defaults.restitution = 0.05f;
			break;
		case 2: // Medium
			defaults.density = 1.0f;
			defaults.friction = 0.6f;
			defaults.restitution = 0.3f;
			break;
		case 3: // Bouncy
			defaults.density = 1.0f;
			defaults.friction = 0.2f;
			defaults.restitution = 0.85f;
			break;
		default: // Custom
			break;
	}
}

void writeScaleChannels( CHOP_Output* output, const std::vector<SpawnBody>& defs )
{
	constexpr float kMinRenderScale = 0.001f;
	auto sanitizeScale = [&]( float v ) {
		float s = fabsf( v );
		return s >= kMinRenderScale ? s : kMinRenderScale;
	};

	for ( int i = 0; i < output->numSamples; ++i )
	{
		float sx = 1.0f;
		float sy = 1.0f;
		float sz = 1.0f;

		if ( i < (int)defs.size() )
		{
			sx = sanitizeScale( defs[i].sizeX );
			sy = sanitizeScale( defs[i].sizeY );
			sz = sanitizeScale( defs[i].sizeZ );
		}

		output->channels[6][i] = sx;
		output->channels[7][i] = sy;
		output->channels[8][i] = sz;
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
	customInfo.opType->setString( "Box3dinstances" );
	customInfo.opLabel->setString( "Box3D Instances" );
	// Max 3 chars, letters only (no digits allowed by TD)
	customInfo.opIcon->setString( "INS" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase* CreateCHOPInstance( const OP_NodeInfo* info )
{
	return new Box3DBodiesCHOP( info );
}

DLLEXPORT
void DestroyCHOPInstance( CHOP_CPlusPlusBase* instance )
{
	delete (Box3DBodiesCHOP*)instance;
}

} // extern "C"

Box3DBodiesCHOP::Box3DBodiesCHOP( const OP_NodeInfo* info ) : myOpId( info->opId )
{
}

Box3DBodiesCHOP::~Box3DBodiesCHOP()
{
	unregisterGroup();
}

void Box3DBodiesCHOP::unregisterGroup()
{
	if ( myGroupRegistered && mySolverOpId != 0 )
	{
		SolverCore* core = Registry::find( mySolverOpId );
		if ( core != nullptr )
		{
			core->removeGroup( myOpId );
		}
	}
	myGroupRegistered = false;
}

void Box3DBodiesCHOP::getGeneralInfo( CHOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// Follows the simulation every frame while its output is in use.
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = false;
}

bool Box3DBodiesCHOP::getOutputInfo( CHOP_OutputInfo* info, const OP_Inputs* inputs, void* )
{
	const OP_SOPInput* sop = inputs->getParSOP( SpawnsopParName );
	int sampleCount = sop != nullptr ? sop->getNumPoints() : 0;

	info->numChannels = kInstancesNumOutputChannels;
	info->numSamples = sampleCount > 0 ? sampleCount : 1;
	info->startIndex = 0;
	return true;
}

void Box3DBodiesCHOP::getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* )
{
	name->setString( kInstancesOutputChannelNames[index] );
}

void Box3DBodiesCHOP::execute( CHOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();

	// Reading the solver CHOP creates the cook dependency: TD cooks (and
	// steps) the solver before this node.
	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	const OP_SOPInput* spawnSop = inputs->getParSOP( SpawnsopParName );

	if ( solverInput == nullptr )
	{
		myWarning = "Set the Solver parameter to a Box3D Solver CHOP.";
		unregisterGroup();
		mySolverOpId = 0;
		myLastSpawnDefs.clear();
		writeTransformChannels( output, nullptr, myOpId );
		writeScaleChannels( output, myLastSpawnDefs );
		return;
	}

	uint32_t solverOpId = solverInput->opId;
	SolverCore* core = Registry::find( solverOpId );
	if ( core == nullptr )
	{
		myWarning = "The Solver parameter does not point to a Box3D Solver CHOP.";
		unregisterGroup();
		mySolverOpId = 0;
		myLastSpawnDefs.clear();
		writeTransformChannels( output, nullptr, myOpId );
		writeScaleChannels( output, myLastSpawnDefs );
		return;
	}

	// Re-bind if the solver changed
	if ( solverOpId != mySolverOpId )
	{
		unregisterGroup();
		mySolverOpId = solverOpId;
	}
	SpawnDefaults defaults = readSpawnDefaults( inputs );
	applyMaterialPreset( inputs->getParInt( MaterialpresetName ), defaults );

	if ( myResetPending )
	{
		core->removeGroup( myOpId );
		myGroupRegistered = false;
		myLastSpawnDefs.clear();
		mySopId = 0;
		mySopCooks = -1;
		myResetPending = false;
	}

	uint32_t sopId = spawnSop != nullptr ? spawnSop->opId : 0;
	int64_t sopCooks = spawnSop != nullptr ? spawnSop->totalCooks : -1;

	bool groupChanged = !myGroupRegistered || defaults != myLastDefaults || sopId != mySopId ||
						( spawnSop != nullptr && sopCooks != mySopCooks );

	if ( groupChanged )
	{
		myLastSpawnDefs = parseSpawnSop( spawnSop, defaults );
		core->setGroup( myOpId, myLastSpawnDefs );
		myGroupRegistered = true;
		myLastDefaults = defaults;
		mySopId = sopId;
		mySopCooks = sopCooks;
	}

	if ( spawnSop == nullptr )
	{
		myWarning = "Set the Spawn SOP parameter (each point spawns one body).";
	}

	// The solver stepped during its own cook; just read our group's state.
	// A freshly registered group is reported at its spawn poses until the
	// solver rebuilds the world on its next cook (one frame of latency).
	writeTransformChannels( output, core, myOpId );
	writeScaleChannels( output, myLastSpawnDefs );
}

int32_t Box3DBodiesCHOP::getNumInfoCHOPChans( void* )
{
	return 1;
}

void Box3DBodiesCHOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	if ( index == 0 )
	{
		chan->name->setString( "body_count" );
		SolverCore* core = mySolverOpId != 0 ? Registry::find( mySolverOpId ) : nullptr;
		chan->value = core != nullptr ? (float)core->getGroupBodyCount( myOpId ) : 0.0f;
	}
}

void Box3DBodiesCHOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
	{
		warning->setString( myWarning.c_str() );
	}
}

void Box3DBodiesCHOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Bodies";
		manager->appendCHOP( p );
	}

	{
		OP_NumericParameter p;
		p.name = ResetName;
		p.label = "Reset";
		p.page = "Bodies";
		manager->appendPulse( p );
	}

	{
		OP_StringParameter p;
		p.name = MaterialpresetName;
		p.label = "Material Preset";
		p.page = "Bodies";
		p.defaultValue = "custom";
		const char* names[] = { "custom", "soft", "medium", "bouncy" };
		const char* labels[] = { "Custom", "Soft", "Medium", "Bouncy" };
		manager->appendMenu( p, 4, names, labels );
	}

	appendBodyParameters( manager, "Bodies" );
}

void Box3DBodiesCHOP::pulsePressed( const char* name, void* )
{
	if ( std::strcmp( name, ResetName ) == 0 )
	{
		myResetPending = true;
	}
}

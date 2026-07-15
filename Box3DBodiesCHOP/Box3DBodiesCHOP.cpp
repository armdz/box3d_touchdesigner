#include "Box3DBodiesCHOP.h"

#include <cmath>
#include <cstring>

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char ResetName[] = "Reset";
constexpr char MaterialpresetName[] = "Materialpreset";
constexpr char ExtrachannelsName[] = "Extrachannels";
constexpr char ContactchannelsName[] = "Contactchannels";
constexpr int kInstancesNumOutputChannels = 9;
constexpr int kInstancesNumExtendedChannels = 16;
// vx vy vz in units/s, wx wy wz in deg/s, awake 1/0
constexpr const char* kInstancesOutputChannelNames[kInstancesNumExtendedChannels] = {
	"tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz", "vx", "vy", "vz", "wx", "wy", "wz", "awake"
};
// touching = live contact count, impulse = summed normal impulse (N*s),
// hitspeed = strongest impact speed captured in the solver's last advance
// (nonzero for exactly one frame per hit — ideal to trigger flashes/sounds).
constexpr int kInstancesNumContactChannels = 3;
constexpr const char* kInstancesContactChannelNames[kInstancesNumContactChannels] = { "touching", "impulse",
																					  "hitspeed" };

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
			const SpawnBody& d = defs[i];
			sx = sanitizeScale( d.sizeX );
			sy = sanitizeScale( d.sizeY );
			sz = sanitizeScale( d.sizeZ );

			// Match the collision shape: spheres only use sizeX (diameter) and
			// capsules are round in XZ, so mirror that in the render scale.
			if ( d.shape == 1 )
			{
				sy = sx;
				sz = sx;
			}
			else if ( d.shape == 2 )
			{
				sz = sx;
			}
		}

		output->channels[6][i] = sx;
		output->channels[7][i] = sy;
		output->channels[8][i] = sz;
	}
}

// Fills vx vy vz wx wy wz awake (channels 9..15) when the Extra Channels
// toggle is on. Angular velocity converted to deg/s for TD friendliness.
void writeStateChannels( CHOP_Output* output, SolverCore* core, uint32_t groupKey )
{
	if ( output->numChannels < kInstancesNumExtendedChannels )
	{
		return;
	}

	constexpr float kRadToDeg = 57.295779513082320877f;

	int written = 0;
	if ( core != nullptr && output->numSamples > 0 )
	{
		std::vector<BodyState> states( output->numSamples );
		written = core->getGroupStates( groupKey, states.data(), output->numSamples );

		for ( int i = 0; i < written; ++i )
		{
			const BodyState& s = states[i];
			output->channels[9][i] = s.vx;
			output->channels[10][i] = s.vy;
			output->channels[11][i] = s.vz;
			output->channels[12][i] = s.wx * kRadToDeg;
			output->channels[13][i] = s.wy * kRadToDeg;
			output->channels[14][i] = s.wz * kRadToDeg;
			output->channels[15][i] = s.awake;
		}
	}

	for ( int i = written; i < output->numSamples; ++i )
	{
		for ( int c = 9; c < kInstancesNumExtendedChannels; ++c )
		{
			output->channels[c][i] = 0.0f;
		}
	}
}

// Fills touching/impulse/hitspeed starting at `offset` when the Contact
// Channels toggle is on.
void writeContactChannels( CHOP_Output* output, SolverCore* core, uint32_t groupKey, int offset )
{
	if ( output->numChannels < offset + kInstancesNumContactChannels )
	{
		return;
	}

	int written = 0;
	if ( core != nullptr && output->numSamples > 0 )
	{
		std::vector<BodyContactState> states( output->numSamples );
		written = core->getGroupContactStates( groupKey, states.data(), output->numSamples );

		for ( int i = 0; i < written; ++i )
		{
			output->channels[offset][i] = states[i].touching;
			output->channels[offset + 1][i] = states[i].impulse;
			output->channels[offset + 2][i] = states[i].hitSpeed;
		}
	}

	for ( int i = written; i < output->numSamples; ++i )
	{
		for ( int c = 0; c < kInstancesNumContactChannels; ++c )
		{
			output->channels[offset + c][i] = 0.0f;
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
	customInfo.opType->setString( "Box3dinstances" );
	customInfo.opLabel->setString( "Box3D Instances" );
	// Max 3 chars, letters only (no digits allowed by TD)
	customInfo.opIcon->setString( "INS" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
	// Kick-start cookEveryFrame on scene load (see getGeneralInfo).
	customInfo.cookOnStart = true;
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

Box3DBodiesCHOP::Box3DBodiesCHOP( const OP_NodeInfo* info ) : myOpId( info->opId ), myNodeInfo( info )
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
	// Cook unconditionally: this node's cook keeps its group registered (and
	// its heartbeat alive) in the physics world. Bypassing the node stops
	// its cooking and the solver drops the group after a few silent steps.
	ginfo->cookEveryFrame = true;
	ginfo->timeslice = false;
}

bool Box3DBodiesCHOP::getOutputInfo( CHOP_OutputInfo* info, const OP_Inputs* inputs, void* )
{
	const OP_SOPInput* sop = inputs->getParSOP( SpawnsopParName );
	int sampleCount = sop != nullptr ? sop->getNumPoints() : 0;

	bool extra = inputs->getParInt( ExtrachannelsName ) != 0;
	bool contact = inputs->getParInt( ContactchannelsName ) != 0;
	info->numChannels = ( extra ? kInstancesNumExtendedChannels : kInstancesNumOutputChannels ) +
						( contact ? kInstancesNumContactChannels : 0 );
	info->numSamples = sampleCount > 0 ? sampleCount : 1;
	info->startIndex = 0;
	return true;
}

void Box3DBodiesCHOP::getChannelName( int32_t index, OP_String* name, const OP_Inputs* inputs, void* )
{
	// The contact channels sit right after whichever transform block is
	// active, so their index depends on the Extra Channels toggle.
	int base = inputs->getParInt( ExtrachannelsName ) != 0 ? kInstancesNumExtendedChannels
														   : kInstancesNumOutputChannels;
	if ( index < base )
	{
		name->setString( kInstancesOutputChannelNames[index] );
	}
	else
	{
		name->setString( kInstancesContactChannelNames[index - base] );
	}
}

void Box3DBodiesCHOP::execute( CHOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();

	// Reading the solver CHOP creates the cook dependency: TD cooks (and
	// steps) the solver before this node.
	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	const OP_SOPInput* spawnSop = inputs->getParSOP( SpawnsopParName );

	bool contactChannels = inputs->getParInt( ContactchannelsName ) != 0;
	int contactBase = inputs->getParInt( ExtrachannelsName ) != 0 ? kInstancesNumExtendedChannels
																  : kInstancesNumOutputChannels;

	if ( solverInput == nullptr )
	{
		myWarning = "Set the Solver parameter to a Box3D Solver CHOP.";
		unregisterGroup();
		mySolverOpId = 0;
		myLastSpawnDefs.clear();
		writeTransformChannels( output, nullptr, myOpId );
		writeScaleChannels( output, myLastSpawnDefs );
		writeStateChannels( output, nullptr, myOpId );
		if ( contactChannels )
		{
			writeContactChannels( output, nullptr, myOpId, contactBase );
		}
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
		writeStateChannels( output, nullptr, myOpId );
		if ( contactChannels )
		{
			writeContactChannels( output, nullptr, myOpId, contactBase );
		}
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

	// Keep the node path registered (it changes on rename/move) so the
	// solver's Joints DAT can reference this group.
	core->setGroupPath( myOpId, myNodeInfo->opPath );

	if ( spawnSop == nullptr )
	{
		myWarning = "Set the Spawn SOP parameter (each point spawns one body).";
	}

	// The solver stepped during its own cook; just read our group's state.
	// A freshly registered group is reported at its spawn poses until the
	// solver rebuilds the world on its next cook (one frame of latency).
	writeTransformChannels( output, core, myOpId );
	writeScaleChannels( output, myLastSpawnDefs );
	writeStateChannels( output, core, myOpId );
	if ( contactChannels )
	{
		writeContactChannels( output, core, myOpId, contactBase );
	}
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
		// Auto-bind: a sibling solver with TD's default name resolves on
		// creation (paths are relative to this node).
		p.defaultValue = "Box3dsolver1";
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
		OP_NumericParameter p;
		p.name = ExtrachannelsName;
		p.label = "Extra Channels (Velocity, Awake)";
		p.page = "Bodies";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = ContactchannelsName;
		p.label = "Contact Channels (Touching, Impulse, Hit)";
		p.page = "Bodies";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
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

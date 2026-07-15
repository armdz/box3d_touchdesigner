#include "Box3DContactsCHOP.h"

#include <cstdio>
#include <cstring>

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char BegineventsName[] = "Beginevents";
constexpr char EndeventsName[] = "Endevents";
constexpr char HiteventsName[] = "Hitevents";
constexpr char FilterbodyName[] = "Filterbody";

constexpr int kNumChans = 13;
constexpr const char* kChanNames[kNumChans] = { "active", "kind", "idxa", "idxb", "worlda", "worldb", "px",
												"py",	  "pz",	  "nx",	  "ny",	  "nz",		"speed" };

const char* kindName( int kind )
{
	switch ( kind )
	{
		case 0:
			return "begin";
		case 1:
			return "end";
		default:
			return "hit";
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
	customInfo.opType->setString( "Box3dcontacts" );
	customInfo.opLabel->setString( "Box3D Contacts" );
	// Max 3 chars, letters only (no digits allowed by TD)
	customInfo.opIcon->setString( "CON" );
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
	return new Box3DContactsCHOP( info );
}

DLLEXPORT
void DestroyCHOPInstance( CHOP_CPlusPlusBase* instance )
{
	delete (Box3DContactsCHOP*)instance;
}

} // extern "C"

Box3DContactsCHOP::Box3DContactsCHOP( const OP_NodeInfo* info ) : myOpId( info->opId )
{
}

Box3DContactsCHOP::~Box3DContactsCHOP()
{
}

void Box3DContactsCHOP::getGeneralInfo( CHOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// Events live exactly one frame in the solver; skipping a cook would drop
	// them silently, so this node observes every frame.
	ginfo->cookEveryFrame = true;
	ginfo->timeslice = false;
}

void Box3DContactsCHOP::fetchEvents( const OP_Inputs* inputs )
{
	myEvents.clear();
	myBeginCount = 0;
	myEndCount = 0;
	myHitCount = 0;
	myWarning.clear();

	// Reading the solver CHOP creates the cook dependency: TD cooks (and
	// steps) the solver before this node, so the event buffer is this frame's.
	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	if ( solverInput == nullptr )
	{
		myWarning = "Set the Solver parameter to a Box3D Solver CHOP.";
		mySolverOpId = 0;
		return;
	}

	SolverCore* core = Registry::find( solverInput->opId );
	if ( core == nullptr )
	{
		myWarning = "The Solver parameter does not point to a Box3D Solver CHOP.";
		mySolverOpId = 0;
		return;
	}
	mySolverOpId = solverInput->opId;

	bool wantBegin = inputs->getParInt( BegineventsName ) != 0;
	bool wantEnd = inputs->getParInt( EndeventsName ) != 0;
	bool wantHit = inputs->getParInt( HiteventsName ) != 0;

	const char* filter = inputs->getParString( FilterbodyName );
	bool hasFilter = filter != nullptr && filter[0] != '\0';
	uint32_t filterKey = 0;
	if ( hasFilter && !core->findGroupKeyByPath( filter, filterKey ) )
	{
		myWarning = "Body filter does not match any registered Body SOP / Instances CHOP.";
		return;
	}

	int count = core->contactEventCount();
	if ( count <= 0 )
	{
		return;
	}

	std::vector<ContactEvent> raw( count );
	count = core->getContactEvents( raw.data(), count );

	myEvents.reserve( count );
	for ( int i = 0; i < count; ++i )
	{
		ContactEvent e = raw[i];

		if ( ( e.kind == 0 && !wantBegin ) || ( e.kind == 1 && !wantEnd ) || ( e.kind == 2 && !wantHit ) )
		{
			continue;
		}

		if ( hasFilter )
		{
			bool matchA = e.groupA == filterKey && e.indexA >= 0;
			bool matchB = e.groupB == filterKey && e.indexB >= 0;
			if ( !matchA && !matchB )
			{
				continue;
			}
			// Normalize: the filtered body is always side A, so idxa indexes
			// straight into that node's instances. Normal points A->B.
			if ( !matchA )
			{
				std::swap( e.groupA, e.groupB );
				std::swap( e.indexA, e.indexB );
				e.nx = -e.nx;
				e.ny = -e.ny;
				e.nz = -e.nz;
			}
		}

		if ( e.kind == 0 )
		{
			++myBeginCount;
		}
		else if ( e.kind == 1 )
		{
			++myEndCount;
		}
		else
		{
			++myHitCount;
		}
		myEvents.push_back( e );
	}
}

bool Box3DContactsCHOP::getOutputInfo( CHOP_OutputInfo* info, const OP_Inputs* inputs, void* )
{
	fetchEvents( inputs );

	info->numChannels = kNumChans;
	info->numSamples = myEvents.empty() ? 1 : (int)myEvents.size();
	info->startIndex = 0;
	return true;
}

void Box3DContactsCHOP::getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* )
{
	name->setString( kChanNames[index] );
}

void Box3DContactsCHOP::execute( CHOP_Output* output, const OP_Inputs* inputs, void* )
{
	// Keep the solver dependency alive on this cook path too.
	inputs->getParCHOP( SolverName );

	for ( int i = 0; i < output->numSamples; ++i )
	{
		if ( i < (int)myEvents.size() )
		{
			const ContactEvent& e = myEvents[i];
			output->channels[0][i] = 1.0f;
			output->channels[1][i] = (float)e.kind;
			output->channels[2][i] = (float)e.indexA;
			output->channels[3][i] = (float)e.indexB;
			output->channels[4][i] = e.indexA < 0 ? 1.0f : 0.0f;
			output->channels[5][i] = e.indexB < 0 ? 1.0f : 0.0f;
			output->channels[6][i] = e.px;
			output->channels[7][i] = e.py;
			output->channels[8][i] = e.pz;
			output->channels[9][i] = e.nx;
			output->channels[10][i] = e.ny;
			output->channels[11][i] = e.nz;
			output->channels[12][i] = e.speed;
		}
		else
		{
			for ( int c = 0; c < kNumChans; ++c )
			{
				output->channels[c][i] = 0.0f;
			}
		}
	}
}

int32_t Box3DContactsCHOP::getNumInfoCHOPChans( void* )
{
	return 4;
}

void Box3DContactsCHOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	if ( index == 0 )
	{
		chan->name->setString( "num_events" );
		chan->value = (float)myEvents.size();
	}
	else if ( index == 1 )
	{
		chan->name->setString( "begin_count" );
		chan->value = (float)myBeginCount;
	}
	else if ( index == 2 )
	{
		chan->name->setString( "end_count" );
		chan->value = (float)myEndCount;
	}
	else
	{
		chan->name->setString( "hit_count" );
		chan->value = (float)myHitCount;
	}
}

bool Box3DContactsCHOP::getInfoDATSize( OP_InfoDATSize* infoSize, void* )
{
	infoSize->rows = (int)myEvents.size() + 1; // + header
	infoSize->cols = 10;
	infoSize->byColumn = false;
	return true;
}

void Box3DContactsCHOP::getInfoDATEntries( int32_t index, int32_t nEntries, OP_InfoDATEntries* entries, void* )
{
	if ( index == 0 )
	{
		const char* headers[10] = { "index", "kind", "body_a", "index_a", "body_b",
									"index_b", "speed", "tx", "ty", "tz" };
		for ( int c = 0; c < nEntries && c < 10; ++c )
		{
			entries->values[c]->setString( headers[c] );
		}
		return;
	}

	int row = index - 1;
	if ( row < 0 || row >= (int)myEvents.size() )
	{
		for ( int c = 0; c < nEntries; ++c )
		{
			entries->values[c]->setString( "" );
		}
		return;
	}

	const ContactEvent& e = myEvents[row];
	SolverCore* core = mySolverOpId != 0 ? Registry::find( mySolverOpId ) : nullptr;

	auto bodyName = []( SolverCore* c, uint32_t group, int idx, std::string& out ) {
		if ( idx < 0 || group == 0 )
		{
			out = "world";
			return;
		}
		if ( c == nullptr || !c->getGroupPathByKey( group, out ) || out.empty() )
		{
			out = "?";
		}
	};

	std::string pathA, pathB;
	bodyName( core, e.groupA, e.indexA, pathA );
	bodyName( core, e.groupB, e.indexB, pathB );

	char buf[64];
	for ( int c = 0; c < nEntries; ++c )
	{
		switch ( c )
		{
			case 0:
				snprintf( buf, sizeof( buf ), "%d", row );
				entries->values[c]->setString( buf );
				break;
			case 1:
				entries->values[c]->setString( kindName( e.kind ) );
				break;
			case 2:
				entries->values[c]->setString( pathA.c_str() );
				break;
			case 3:
				snprintf( buf, sizeof( buf ), "%d", e.indexA );
				entries->values[c]->setString( buf );
				break;
			case 4:
				entries->values[c]->setString( pathB.c_str() );
				break;
			case 5:
				snprintf( buf, sizeof( buf ), "%d", e.indexB );
				entries->values[c]->setString( buf );
				break;
			case 6:
				snprintf( buf, sizeof( buf ), "%.4f", e.speed );
				entries->values[c]->setString( buf );
				break;
			case 7:
				snprintf( buf, sizeof( buf ), "%.4f", e.px );
				entries->values[c]->setString( buf );
				break;
			case 8:
				snprintf( buf, sizeof( buf ), "%.4f", e.py );
				entries->values[c]->setString( buf );
				break;
			default:
				snprintf( buf, sizeof( buf ), "%.4f", e.pz );
				entries->values[c]->setString( buf );
				break;
		}
	}
}

void Box3DContactsCHOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
	{
		warning->setString( myWarning.c_str() );
	}
}

void Box3DContactsCHOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Contacts";
		// Auto-bind: a sibling solver with TD's default name resolves on
		// creation (paths are relative to this node).
		p.defaultValue = "Box3dsolver1";
		manager->appendCHOP( p );
	}

	{
		OP_NumericParameter p;
		p.name = BegineventsName;
		p.label = "Begin Touch Events";
		p.page = "Contacts";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = EndeventsName;
		p.label = "End Touch Events";
		p.page = "Contacts";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = HiteventsName;
		p.label = "Hit Events";
		p.page = "Contacts";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		// Path or node name of a Box3D Body SOP / Instances CHOP: keep only
		// events involving that node, normalized so it is always side A.
		OP_StringParameter p;
		p.name = FilterbodyName;
		p.label = "Body Filter";
		p.page = "Contacts";
		p.defaultValue = "";
		manager->appendString( p );
	}
}

#include "Box3DForceCHOP.h"

#include <cstring>

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char TypeName[] = "Type";
constexpr char PositionName[] = "Position";
constexpr char PointssopName[] = "Pointssop";
constexpr char DirectionName[] = "Direction";
constexpr char StrengthName[] = "Strength";
constexpr char RadiusName[] = "Radius";
constexpr char FalloffName[] = "Falloff";
constexpr char MassindepName[] = "Massindep";
constexpr char BodiesName[] = "Bodies";

constexpr int kNumChans = 4;
constexpr const char* kChanNames[kNumChans] = { "tx", "ty", "tz", "strength" };

int pointCountOf( const OP_Inputs* inputs )
{
	const OP_SOPInput* sop = inputs->getParSOP( PointssopName );
	return sop != nullptr ? sop->getNumPoints() : 0;
}

} // namespace

extern "C"
{

DLLEXPORT
void FillCHOPPluginInfo( CHOP_PluginInfo* info )
{
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString( "Box3dforce" );
	customInfo.opLabel->setString( "Box3D Force" );
	// Max 3 chars, letters only (no digits allowed by TD)
	customInfo.opIcon->setString( "FRC" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
	customInfo.cookOnStart = true;
}

DLLEXPORT
CHOP_CPlusPlusBase* CreateCHOPInstance( const OP_NodeInfo* info )
{
	return new Box3DForceCHOP( info );
}

DLLEXPORT
void DestroyCHOPInstance( CHOP_CPlusPlusBase* instance )
{
	delete (Box3DForceCHOP*)instance;
}

} // extern "C"

Box3DForceCHOP::Box3DForceCHOP( const OP_NodeInfo* info ) : myOpId( info->opId )
{
}

Box3DForceCHOP::~Box3DForceCHOP()
{
	unregister();
}

void Box3DForceCHOP::unregister()
{
	if ( myRegistered && mySolverOpId != 0 )
	{
		SolverCore* core = Registry::find( mySolverOpId );
		if ( core != nullptr )
		{
			core->removeForceNode( myOpId );
		}
	}
	myRegistered = false;
}

void Box3DForceCHOP::getGeneralInfo( CHOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// Cook unconditionally: this node's cook keeps its force field registered.
	// Bypassing the node turns the force off after a few silent steps.
	ginfo->cookEveryFrame = true;
	ginfo->timeslice = false;
}

bool Box3DForceCHOP::getOutputInfo( CHOP_OutputInfo* info, const OP_Inputs* inputs, void* )
{
	int fields = pointCountOf( inputs );
	info->numChannels = kNumChans;
	info->numSamples = fields > 0 ? fields : 1;
	info->startIndex = 0;
	return true;
}

void Box3DForceCHOP::getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* )
{
	name->setString( kChanNames[index] );
}

void Box3DForceCHOP::execute( CHOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();
	myFields.clear();

	int type = inputs->getParInt( TypeName );
	inputs->enablePar( DirectionName, type == 2 || type == 3 );		// wind / vortex
	inputs->enablePar( RadiusName, type != 2 );						// wind ignores radius
	inputs->enablePar( FalloffName, type != 2 );
	const OP_SOPInput* pointsSop = inputs->getParSOP( PointssopName );
	bool hasPoints = pointsSop != nullptr && pointsSop->getNumPoints() > 0;
	inputs->enablePar( PositionName, !hasPoints );

	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	SolverCore* core = solverInput != nullptr ? Registry::find( solverInput->opId ) : nullptr;

	if ( core == nullptr )
	{
		myWarning = solverInput == nullptr ? "Set the Solver parameter to a Box3D Solver CHOP."
										   : "The Solver parameter does not point to a Box3D Solver CHOP.";
		unregister();
		mySolverOpId = 0;
		for ( int i = 0; i < output->numSamples; ++i )
			for ( int c = 0; c < kNumChans; ++c )
				output->channels[c][i] = 0.0f;
		return;
	}

	if ( solverInput->opId != mySolverOpId )
	{
		unregister();
		mySolverOpId = solverInput->opId;
	}

	// Shared field settings.
	ForceField base;
	base.type = type;
	double dx, dy, dz;
	inputs->getParDouble3( DirectionName, dx, dy, dz );
	base.dx = (float)dx;
	base.dy = (float)dy;
	base.dz = (float)dz;
	base.strength = (float)inputs->getParDouble( StrengthName );
	base.radius = (float)inputs->getParDouble( RadiusName );
	base.falloff = inputs->getParInt( FalloffName );
	base.useMass = inputs->getParInt( MassindepName ) == 0; // toggle is "mass independent"

	// Target: all bodies, or one Body SOP / Instances node by path/name. A
	// non-empty filter that resolves to nothing must apply the force to NOTHING
	// (leaving targetGroup at 0 would silently blast every body in the world).
	const char* bodies = inputs->getParString( BodiesName );
	if ( bodies != nullptr && bodies[0] != '\0' )
	{
		uint32_t key = 0;
		if ( core->findGroupKeyByPath( bodies, key ) )
		{
			base.targetGroup = key;
		}
		else
		{
			myWarning = "Bodies filter does not match any registered Body SOP / Instances node.";
			myFields.clear();
			core->removeForceNode( myOpId );
			myRegistered = false;
			for ( int i = 0; i < output->numSamples; ++i )
				for ( int c = 0; c < kNumChans; ++c )
					output->channels[c][i] = 0.0f;
			return;
		}
	}

	// Build the field list: one per SOP point, or a single field at Position.
	if ( hasPoints )
	{
		int n = pointsSop->getNumPoints();
		const Position* pts = pointsSop->getPointPositions();
		myFields.reserve( n );
		for ( int i = 0; i < n; ++i )
		{
			ForceField f = base;
			f.px = pts[i].x;
			f.py = pts[i].y;
			f.pz = pts[i].z;
			myFields.push_back( f );
		}
	}
	else
	{
		double px, py, pz;
		inputs->getParDouble3( PositionName, px, py, pz );
		ForceField f = base;
		f.px = (float)px;
		f.py = (float)py;
		f.pz = (float)pz;
		myFields.push_back( f );
	}

	core->setForceNodeList( myOpId, myFields );
	myRegistered = true;

	// Output: field positions + strength, one sample per field.
	for ( int i = 0; i < output->numSamples; ++i )
	{
		if ( i < (int)myFields.size() )
		{
			output->channels[0][i] = myFields[i].px;
			output->channels[1][i] = myFields[i].py;
			output->channels[2][i] = myFields[i].pz;
			output->channels[3][i] = myFields[i].strength;
		}
		else
		{
			for ( int c = 0; c < kNumChans; ++c )
				output->channels[c][i] = 0.0f;
		}
	}
}

int32_t Box3DForceCHOP::getNumInfoCHOPChans( void* )
{
	return 1;
}

void Box3DForceCHOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	if ( index == 0 )
	{
		chan->name->setString( "field_count" );
		chan->value = (float)myFields.size();
	}
}

void Box3DForceCHOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
	{
		warning->setString( myWarning.c_str() );
	}
}

void Box3DForceCHOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Force";
		p.defaultValue = "Box3dsolver1";
		manager->appendCHOP( p );
	}

	{
		OP_StringParameter p;
		p.name = TypeName;
		p.label = "Type";
		p.page = "Force";
		p.defaultValue = "attractor";
		const char* names[] = { "attractor", "repulsor", "wind", "vortex" };
		const char* labels[] = { "Attractor", "Repulsor", "Wind (Directional)", "Vortex" };
		manager->appendMenu( p, 4, names, labels );
	}

	{
		OP_NumericParameter p;
		p.name = PositionName;
		p.label = "Position";
		p.page = "Force";
		for ( int i = 0; i < 3; ++i )
		{
			p.defaultValues[i] = i == 1 ? 3.0 : 0.0;
			p.minSliders[i] = -10.0;
			p.maxSliders[i] = 10.0;
		}
		manager->appendXYZ( p );
	}

	{
		// Each point becomes one field at its position (many attractors at once).
		OP_StringParameter p;
		p.name = PointssopName;
		p.label = "Points SOP";
		p.page = "Force";
		manager->appendSOP( p );
	}

	{
		OP_NumericParameter p;
		p.name = DirectionName;
		p.label = "Direction / Axis";
		p.page = "Force";
		p.defaultValues[0] = 0.0;
		p.defaultValues[1] = 1.0;
		p.defaultValues[2] = 0.0;
		for ( int i = 0; i < 3; ++i )
		{
			p.minSliders[i] = -1.0;
			p.maxSliders[i] = 1.0;
		}
		manager->appendXYZ( p );
	}

	{
		// Negative flips an attractor into a repulsor and vice versa.
		OP_NumericParameter p;
		p.name = StrengthName;
		p.label = "Strength";
		p.page = "Force";
		p.defaultValues[0] = 10.0;
		p.minSliders[0] = -50.0;
		p.maxSliders[0] = 50.0;
		manager->appendFloat( p );
	}

	{
		// 0 = unlimited reach.
		OP_NumericParameter p;
		p.name = RadiusName;
		p.label = "Radius (0 = Infinite)";
		p.page = "Force";
		p.defaultValues[0] = 0.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 20.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_StringParameter p;
		p.name = FalloffName;
		p.label = "Falloff";
		p.page = "Force";
		p.defaultValue = "none";
		const char* names[] = { "none", "linear", "invsq" };
		const char* labels[] = { "None", "Linear (to Radius)", "Inverse Square" };
		manager->appendMenu( p, 3, names, labels );
	}

	{
		// On = every body accelerates equally (like gravity). Off = raw force,
		// so heavier bodies respond less.
		OP_NumericParameter p;
		p.name = MassindepName;
		p.label = "Mass Independent";
		p.page = "Force";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		// Empty = all dynamic bodies; else a Body SOP / Instances path or name.
		OP_StringParameter p;
		p.name = BodiesName;
		p.label = "Bodies (Empty = All)";
		p.page = "Force";
		p.defaultValue = "";
		manager->appendString( p );
	}
}

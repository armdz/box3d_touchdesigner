#include "Box3DDebugSOP.h"

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char BodiesName[] = "Bodies";
constexpr char WorldcollisionName[] = "Worldcollision";
constexpr char JointsName[] = "Joints";

} // namespace

extern "C"
{

DLLEXPORT
void FillSOPPluginInfo( SOP_PluginInfo* info )
{
	info->apiVersion = SOPCPlusPlusAPIVersion;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString( "Box3ddebug" );
	customInfo.opLabel->setString( "Box3D Debug" );
	customInfo.opIcon->setString( "DBG" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
}

DLLEXPORT
SOP_CPlusPlusBase* CreateSOPInstance( const OP_NodeInfo* info )
{
	return new Box3DDebugSOP( info );
}

DLLEXPORT
void DestroySOPInstance( SOP_CPlusPlusBase* instance )
{
	delete (Box3DDebugSOP*)instance;
}

} // extern "C"

Box3DDebugSOP::Box3DDebugSOP( const OP_NodeInfo* )
{
}

Box3DDebugSOP::~Box3DDebugSOP()
{
}

void Box3DDebugSOP::getGeneralInfo( SOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// Follows the simulation every frame while its output is in use.
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->directToGPU = false;
}

void Box3DDebugSOP::execute( SOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();
	mySegmentCount = 0;

	// Reading the solver CHOP creates the cook dependency: TD cooks (and
	// steps) the solver before this node.
	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	SolverCore* core = solverInput != nullptr ? Registry::find( solverInput->opId ) : nullptr;

	if ( core == nullptr )
	{
		myWarning = solverInput == nullptr ? "Set the Solver parameter to a Box3D Solver CHOP."
										   : "The Solver parameter does not point to a Box3D Solver CHOP.";
		return; // empty output
	}

	bool bodies = inputs->getParInt( BodiesName ) != 0;
	bool worldCollision = inputs->getParInt( WorldcollisionName ) != 0;
	bool joints = inputs->getParInt( JointsName ) != 0;

	std::vector<float> segments;
	std::vector<float> colors;
	core->getDebugWireframe( bodies, worldCollision, joints, segments, colors );

	mySegmentCount = (int)( segments.size() / 6 );
	if ( mySegmentCount == 0 )
	{
		return;
	}

	BoundingBox bbox( Position( segments[0], segments[1], segments[2] ),
					  Position( segments[0], segments[1], segments[2] ) );

	for ( int i = 0; i < mySegmentCount; ++i )
	{
		Position a( segments[i * 6 + 0], segments[i * 6 + 1], segments[i * 6 + 2] );
		Position b( segments[i * 6 + 3], segments[i * 6 + 4], segments[i * 6 + 5] );
		int32_t ia = output->addPoint( a );
		int32_t ib = output->addPoint( b );

		Color c( colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f );
		output->setColor( c, ia );
		output->setColor( c, ib );

		const int32_t line[2] = { ia, ib };
		output->addLine( line, 2 );

		bbox.enlargeBounds( a );
		bbox.enlargeBounds( b );
	}

	output->setBoundingBox( bbox );
}

void Box3DDebugSOP::executeVBO( SOP_VBOOutput*, const OP_Inputs*, void* )
{
	// directToGPU is false; this path is never used
}

int32_t Box3DDebugSOP::getNumInfoCHOPChans( void* )
{
	return 1;
}

void Box3DDebugSOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	if ( index == 0 )
	{
		chan->name->setString( "segments" );
		chan->value = (float)mySegmentCount;
	}
}

void Box3DDebugSOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
	{
		warning->setString( myWarning.c_str() );
	}
}

void Box3DDebugSOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Debug";
		// Auto-bind: a sibling solver with TD's default name resolves on
		// creation (paths are relative to this node).
		p.defaultValue = "Box3dsolver1";
		manager->appendCHOP( p );
	}

	{
		OP_NumericParameter p;
		p.name = BodiesName;
		p.label = "Bodies";
		p.page = "Debug";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = WorldcollisionName;
		p.label = "World Collision (Ground/Walls/Mesh)";
		p.page = "Debug";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = JointsName;
		p.label = "Joints";
		p.page = "Debug";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}
}

#include "Box3DBodySOP.h"

#include "TDB3Common.h"

#include "box3d/math_functions.h"

#include <cmath>

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char ShapeName[] = "Shape";
constexpr char SizeName[] = "Size";
constexpr char PositionName[] = "Position";
constexpr char TypeName[] = "Type";
constexpr char DensityName[] = "Density";
constexpr char FrictionName[] = "Friction";
constexpr char RestitutionName[] = "Restitution";
constexpr char ResetoninputName[] = "Resetoninput";

// Menu index (input hull, box, sphere, capsule) → core SpawnBody::shape
int menuShapeToCoreShape( int menuShape )
{
	switch ( menuShape )
	{
		case 1:
			return 0; // box
		case 2:
			return 1; // sphere
		case 3:
			return 2; // capsule
		default:
			return 3; // input hull
	}
}

b3Matrix3 rotationFromTransform( const BodyTransform& t )
{
	b3Quat q;
	q.v.x = t.qx;
	q.v.y = t.qy;
	q.v.z = t.qz;
	q.s = t.qw;
	return b3MakeMatrixFromQuat( q );
}

Position applyTransform( const b3Matrix3& r, const BodyTransform& t, float lx, float ly, float lz )
{
	return Position( r.cx.x * lx + r.cy.x * ly + r.cz.x * lz + t.px, r.cx.y * lx + r.cy.y * ly + r.cz.y * lz + t.py,
					 r.cx.z * lx + r.cy.z * ly + r.cz.z * lz + t.pz );
}

Vector rotateVector( const b3Matrix3& r, float lx, float ly, float lz )
{
	return Vector( r.cx.x * lx + r.cy.x * ly + r.cz.x * lz, r.cx.y * lx + r.cy.y * ly + r.cz.y * lz,
				   r.cx.z * lx + r.cy.z * ly + r.cz.z * lz );
}

} // namespace

extern "C"
{

DLLEXPORT
void FillSOPPluginInfo( SOP_PluginInfo* info )
{
	info->apiVersion = SOPCPlusPlusAPIVersion;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString( "Box3dbody" );
	customInfo.opLabel->setString( "Box3D Body" );
	// Max 3 chars, letters only (no digits allowed by TD)
	customInfo.opIcon->setString( "BOD" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 1;
}

DLLEXPORT
SOP_CPlusPlusBase* CreateSOPInstance( const OP_NodeInfo* info )
{
	return new Box3DBodySOP( info );
}

DLLEXPORT
void DestroySOPInstance( SOP_CPlusPlusBase* instance )
{
	delete (Box3DBodySOP*)instance;
}

} // extern "C"

Box3DBodySOP::Box3DBodySOP( const OP_NodeInfo* info ) : myOpId( info->opId )
{
}

Box3DBodySOP::~Box3DBodySOP()
{
	unregisterGroup();
}

void Box3DBodySOP::unregisterGroup()
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

void Box3DBodySOP::getGeneralInfo( SOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// Follows the simulation every frame while its output is in use.
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->directToGPU = false;
}

Box3DBodySOP::BodySettings Box3DBodySOP::readSettings( const OP_Inputs* inputs ) const
{
	BodySettings s;
	s.shape = inputs->getParInt( ShapeName );

	double x, y, z;
	inputs->getParDouble3( SizeName, x, y, z );
	s.sizeX = (float)x;
	s.sizeY = (float)y;
	s.sizeZ = (float)z;

	inputs->getParDouble3( PositionName, x, y, z );
	s.posX = (float)x;
	s.posY = (float)y;
	s.posZ = (float)z;

	s.type = inputs->getParInt( TypeName );
	s.density = (float)inputs->getParDouble( DensityName );
	s.friction = (float)inputs->getParDouble( FrictionName );
	s.restitution = (float)inputs->getParDouble( RestitutionName );
	return s;
}

void Box3DBodySOP::execute( SOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();

	const OP_SOPInput* input = inputs->getInputSOP( 0 );
	BodySettings settings = readSettings( inputs );

	// Reading the solver CHOP creates the cook dependency: TD cooks (and
	// steps) the solver before this node.
	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	SolverCore* core = solverInput != nullptr ? Registry::find( solverInput->opId ) : nullptr;

	if ( core == nullptr )
	{
		myWarning = solverInput == nullptr ? "Set the Solver parameter to a Box3D Solver CHOP."
										   : "The Solver parameter does not point to a Box3D Solver CHOP.";
		unregisterGroup();
		mySolverOpId = 0;

		// No simulation: show the input (or the primitive preview) untransformed
		BodyTransform identity = { 0, 0, 0, 0, 0, 0, 1 };
		if ( input != nullptr )
		{
			myCentroid[0] = myCentroid[1] = myCentroid[2] = 0.0f;
			outputTransformedInput( output, input, identity );
		}
		else
		{
			identity.px = settings.posX;
			identity.py = settings.posY;
			identity.pz = settings.posZ;
			outputPrimitiveMesh( output, settings, identity );
		}
		myTransform = identity;
		return;
	}

	uint32_t solverOpId = solverInput->opId;
	if ( solverOpId != mySolverOpId )
	{
		unregisterGroup();
		mySolverOpId = solverOpId;
	}

	inputs->enablePar( ResetoninputName, input != nullptr );
	inputs->enablePar( PositionName, input == nullptr );

	bool resetOnInput = inputs->getParInt( ResetoninputName ) != 0;
	uint32_t sopId = input != nullptr ? input->opId : 0;
	int64_t sopCooks = input != nullptr ? input->totalCooks : -1;

	bool changed = !myGroupRegistered || settings != myLastSettings || sopId != mySopId ||
				   ( input != nullptr && resetOnInput && sopCooks != mySopCooks );

	if ( changed )
	{
		SpawnBody def;
		def.type = 2 - settings.type; // menu dynamic/kinematic/static → core 2/1/0
		def.density = settings.density;
		def.friction = settings.friction;
		def.restitution = settings.restitution;
		def.sizeX = settings.sizeX;
		def.sizeY = settings.sizeY;
		def.sizeZ = settings.sizeZ;

		if ( input != nullptr )
		{
			// Body origin = input centroid; the input pose is the spawn pose
			int pointCount = input->getNumPoints();
			const Position* points = input->getPointPositions();

			double cx = 0.0, cy = 0.0, cz = 0.0;
			for ( int i = 0; i < pointCount; ++i )
			{
				cx += points[i].x;
				cy += points[i].y;
				cz += points[i].z;
			}
			if ( pointCount > 0 )
			{
				cx /= pointCount;
				cy /= pointCount;
				cz /= pointCount;
			}
			myCentroid[0] = (float)cx;
			myCentroid[1] = (float)cy;
			myCentroid[2] = (float)cz;

			def.px = myCentroid[0];
			def.py = myCentroid[1];
			def.pz = myCentroid[2];
			def.shape = menuShapeToCoreShape( settings.shape );

			if ( def.shape == 3 )
			{
				def.hullPoints.reserve( pointCount * 3 );
				for ( int i = 0; i < pointCount; ++i )
				{
					def.hullPoints.push_back( points[i].x - myCentroid[0] );
					def.hullPoints.push_back( points[i].y - myCentroid[1] );
					def.hullPoints.push_back( points[i].z - myCentroid[2] );
				}
			}
		}
		else
		{
			// No input: primitive at the Position parameter ("Input Hull"
			// falls back to a box)
			myCentroid[0] = myCentroid[1] = myCentroid[2] = 0.0f;
			def.px = settings.posX;
			def.py = settings.posY;
			def.pz = settings.posZ;
			int coreShape = menuShapeToCoreShape( settings.shape );
			def.shape = coreShape == 3 ? 0 : coreShape;
		}

		core->setGroup( myOpId, { def } );
		myGroupRegistered = true;
		myLastSettings = settings;
		mySopId = sopId;
		mySopCooks = sopCooks;
	}

	BodyTransform t = { 0, 0, 0, 0, 0, 0, 1 };
	core->getGroupTransforms( myOpId, &t, 1 );
	myTransform = t;

	if ( input != nullptr )
	{
		outputTransformedInput( output, input, t );
	}
	else
	{
		outputPrimitiveMesh( output, myLastSettings, t );
	}
}

void Box3DBodySOP::outputTransformedInput( SOP_Output* output, const OP_SOPInput* input, const BodyTransform& t )
{
	b3Matrix3 r = rotationFromTransform( t );

	int pointCount = input->getNumPoints();
	const Position* points = input->getPointPositions();

	for ( int i = 0; i < pointCount; ++i )
	{
		Position p = applyTransform( r, t, points[i].x - myCentroid[0], points[i].y - myCentroid[1],
									 points[i].z - myCentroid[2] );
		output->addPoint( p );
	}

	if ( input->hasNormals() )
	{
		const SOP_NormalInfo* normalInfo = input->getNormals();
		if ( normalInfo != nullptr && normalInfo->normals != nullptr )
		{
			for ( int i = 0; i < pointCount; ++i )
			{
				const Vector& n = normalInfo->normals[i];
				output->setNormal( rotateVector( r, n.x, n.y, n.z ), i );
			}
		}
	}

	int primCount = input->getNumPrimitives();
	for ( int i = 0; i < primCount; ++i )
	{
		const SOP_PrimitiveInfo& prim = input->getPrimitive( i );
		if ( prim.type != PrimitiveType::Polygon || prim.numVertices < 3 )
		{
			continue;
		}
		for ( int v = 1; v < prim.numVertices - 1; ++v )
		{
			output->addTriangle( prim.pointIndices[0], prim.pointIndices[v], prim.pointIndices[v + 1] );
		}
	}
}

void Box3DBodySOP::outputPrimitiveMesh( SOP_Output* output, const BodySettings& s, const BodyTransform& t )
{
	b3Matrix3 r = rotationFromTransform( t );

	int coreShape = menuShapeToCoreShape( s.shape );
	if ( coreShape == 3 )
	{
		coreShape = 0; // no input: hull previews as a box
	}

	if ( coreShape == 0 )
	{
		// Box: 8 corners, 12 triangles
		float hx = 0.5f * s.sizeX, hy = 0.5f * s.sizeY, hz = 0.5f * s.sizeZ;
		const float corners[8][3] = { { -hx, -hy, -hz }, { hx, -hy, -hz }, { hx, hy, -hz }, { -hx, hy, -hz },
									  { -hx, -hy, hz },	 { hx, -hy, hz },  { hx, hy, hz },	{ -hx, hy, hz } };
		for ( int i = 0; i < 8; ++i )
		{
			output->addPoint( applyTransform( r, t, corners[i][0], corners[i][1], corners[i][2] ) );
			float inv = 1.0f / sqrtf( corners[i][0] * corners[i][0] + corners[i][1] * corners[i][1] +
									  corners[i][2] * corners[i][2] + 1e-12f );
			output->setNormal( rotateVector( r, corners[i][0] * inv, corners[i][1] * inv, corners[i][2] * inv ), i );
		}
		const int32_t tris[36] = { 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
								   2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3 };
		output->addTriangles( tris, 12 );
		return;
	}

	// Sphere and capsule share a lat-long tessellation; the capsule shifts
	// the hemispheres apart along Y.
	float radius = 0.5f * s.sizeX;
	float half = 0.0f;
	if ( coreShape == 2 )
	{
		half = 0.5f * s.sizeY - radius;
		half = half > 0.0f ? half : 0.0f;
	}

	constexpr int kSegments = 24;
	constexpr int kRings = 16;

	int pointIndex = 0;
	for ( int ring = 0; ring <= kRings; ++ring )
	{
		float theta = 3.14159265f * (float)ring / (float)kRings;
		float sy = cosf( theta );
		float sr = sinf( theta );
		float yOffset = sy >= 0.0f ? half : -half;

		for ( int seg = 0; seg <= kSegments; ++seg )
		{
			float phi = 2.0f * 3.14159265f * (float)seg / (float)kSegments;
			float nx = sr * cosf( phi );
			float ny = sy;
			float nz = sr * sinf( phi );

			output->addPoint( applyTransform( r, t, radius * nx, radius * ny + yOffset, radius * nz ) );
			output->setNormal( rotateVector( r, nx, ny, nz ), pointIndex );
			++pointIndex;
		}
	}

	for ( int ring = 0; ring < kRings; ++ring )
	{
		for ( int seg = 0; seg < kSegments; ++seg )
		{
			int a = ring * ( kSegments + 1 ) + seg;
			int b = a + kSegments + 1;
			output->addTriangle( a, a + 1, b );
			output->addTriangle( a + 1, b + 1, b );
		}
	}
}

void Box3DBodySOP::executeVBO( SOP_VBOOutput*, const OP_Inputs*, void* )
{
	// directToGPU is false; this path is never used
}

int32_t Box3DBodySOP::getNumInfoCHOPChans( void* )
{
	return 6;
}

void Box3DBodySOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	static const char* names[6] = { "tx", "ty", "tz", "rx", "ry", "rz" };
	chan->name->setString( names[index] );

	if ( index < 3 )
	{
		const float pos[3] = { myTransform.px, myTransform.py, myTransform.pz };
		chan->value = pos[index];
		return;
	}

	float rx, ry, rz;
	quatToEulerXYZDegrees( myTransform.qx, myTransform.qy, myTransform.qz, myTransform.qw, &rx, &ry, &rz );
	const float rot[3] = { rx, ry, rz };
	chan->value = rot[index - 3];
}

void Box3DBodySOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
	{
		warning->setString( myWarning.c_str() );
	}
}

void Box3DBodySOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Body";
		manager->appendCHOP( p );
	}

	{
		OP_StringParameter p;
		p.name = ShapeName;
		p.label = "Shape";
		p.page = "Body";
		p.defaultValue = "inputhull";
		const char* names[] = { "inputhull", "box", "sphere", "capsule" };
		const char* labels[] = { "Input Hull", "Box", "Sphere", "Capsule" };
		manager->appendMenu( p, 4, names, labels );
	}

	{
		OP_NumericParameter p;
		p.name = SizeName;
		p.label = "Size";
		p.page = "Body";
		for ( int i = 0; i < 3; ++i )
		{
			p.defaultValues[i] = 1.0;
			p.minValues[i] = 0.001;
			p.minSliders[i] = 0.05;
			p.maxSliders[i] = 4.0;
			p.clampMins[i] = true;
		}
		manager->appendXYZ( p );
	}

	{
		OP_NumericParameter p;
		p.name = PositionName;
		p.label = "Position";
		p.page = "Body";
		p.defaultValues[0] = 0.0;
		p.defaultValues[1] = 3.0;
		p.defaultValues[2] = 0.0;
		for ( int i = 0; i < 3; ++i )
		{
			p.minSliders[i] = -10.0;
			p.maxSliders[i] = 10.0;
		}
		manager->appendXYZ( p );
	}

	{
		OP_StringParameter p;
		p.name = TypeName;
		p.label = "Body Type";
		p.page = "Body";
		p.defaultValue = "dynamic";
		const char* names[] = { "dynamic", "kinematic", "static" };
		const char* labels[] = { "Dynamic", "Kinematic", "Static" };
		manager->appendMenu( p, 3, names, labels );
	}

	{
		OP_NumericParameter p;
		p.name = DensityName;
		p.label = "Density";
		p.page = "Body";
		p.defaultValues[0] = 1.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 10.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = FrictionName;
		p.label = "Friction";
		p.page = "Body";
		p.defaultValues[0] = 0.6;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 1.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = RestitutionName;
		p.label = "Restitution";
		p.page = "Body";
		p.defaultValues[0] = 0.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 1.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = ResetoninputName;
		p.label = "Reset On Input Change";
		p.page = "Body";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}
}

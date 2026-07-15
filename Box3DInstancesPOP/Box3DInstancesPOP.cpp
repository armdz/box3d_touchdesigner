#include "Box3DInstancesPOP.h"

#include <cmath>
#include <cstring>

using namespace tdb3;

namespace
{
constexpr char SolverName[] = "Solver";
constexpr char ResetName[] = "Reset";
constexpr char ShapeName[] = "Shape";
constexpr char SizeName[] = "Size";
constexpr char DensityName[] = "Density";
constexpr char FrictionName[] = "Friction";
constexpr char RestitutionName[] = "Restitution";
constexpr char TypeName[] = "Type";
constexpr char BulletName[] = "Bulletccd";
constexpr char KeepAliveName[] = "Keepalive";
constexpr char ExtrachansName[] = "Extrachans";

constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kRadToDeg = 57.295779513082320877f;
constexpr float kMinRenderScale = 0.001f;

// TD-style Euler XYZ (degrees) -> quaternion (x y z w). Rotate order is X then
// Y then Z, matching the rest of the Box3D plugins.
void eulerXYZDegreesToQuat( float rxDeg, float ryDeg, float rzDeg, float& qx, float& qy, float& qz, float& qw )
{
	float hx = 0.5f * rxDeg * kDegToRad;
	float hy = 0.5f * ryDeg * kDegToRad;
	float hz = 0.5f * rzDeg * kDegToRad;
	float cx = cosf( hx ), sx = sinf( hx );
	float cy = cosf( hy ), sy = sinf( hy );
	float cz = cosf( hz ), sz = sinf( hz );
	qw = cx * cy * cz + sx * sy * sz;
	qx = sx * cy * cz - cx * sy * sz;
	qy = cx * sy * cz + sx * cy * sz;
	qz = cx * cy * sz - sx * sy * cz;
}

// Quaternion (x y z w) -> TD Euler XYZ degrees. Same extraction (and rotate
// order) as tdb3::quatToEulerXYZDegrees so it matches the Instances CHOP.
void quatToEulerXYZDegrees( float x, float y, float z, float w, float& rx, float& ry, float& rz )
{
	// Rotation matrix columns (cx, cy, cz), same convention as box3d.
	float cxx = 1.0f - 2.0f * ( y * y + z * z ); // cx.x
	float cxy = 2.0f * ( x * y + w * z );		 // cx.y
	float cxz = 2.0f * ( x * z - w * y );		 // cx.z
	float cyz = 2.0f * ( y * z + w * x );		 // cy.z
	float czz = 1.0f - 2.0f * ( x * x + y * y ); // cz.z

	float sinPitch = -cxz;
	if ( sinPitch >= 0.99999f )
	{
		float cyx = 2.0f * ( x * y - w * z ); // cy.x
		float cyy = 1.0f - 2.0f * ( x * x + z * z ); // cy.y
		rx = atan2f( cyx, cyy ) * kRadToDeg;
		ry = 90.0f;
		rz = 0.0f;
	}
	else if ( sinPitch <= -0.99999f )
	{
		float cyx = 2.0f * ( x * y - w * z );
		float cyy = 1.0f - 2.0f * ( x * x + z * z );
		rx = atan2f( -cyx, cyy ) * kRadToDeg;
		ry = -90.0f;
		rz = 0.0f;
	}
	else
	{
		rx = atan2f( cyz, czz ) * kRadToDeg;
		ry = asinf( sinPitch ) * kRadToDeg;
		rz = atan2f( cxy, cxx ) * kRadToDeg;
	}
}

// A CPU-resident view of one input point attribute. Holds the buffer ref alive.
struct PopAttr
{
	OP_SmartRef<POP_Buffer> buf;
	const void* data = nullptr;
	int comps = 0;
	POP_AttributeType type = POP_AttributeType::Float;
	explicit operator bool() const { return data != nullptr; }
};

PopAttr fetchPointAttr( const OP_POPInput* in, const char* name )
{
	PopAttr r;
	const POP_Attribute* a = in->getAttribute( POP_AttributeClass::Point, name, nullptr );
	if ( a == nullptr )
		return r;
	POP_GetBufferInfo gi;
	gi.location = POP_BufferLocation::CPU;
	r.buf = a->getBuffer( gi, nullptr );
	if ( !r.buf )
		return r;
	r.data = r.buf->getData( nullptr );
	r.comps = (int)a->info.numComponents;
	r.type = a->info.type;
	return r;
}

float readComp( const PopAttr& a, int point, int comp, float def )
{
	if ( !a || comp >= a.comps )
		return def;
	int index = point * a.comps + comp;
	switch ( a.type )
	{
		case POP_AttributeType::Float:
			return ( (const float*)a.data )[index];
		case POP_AttributeType::Double:
			return (float)( (const double*)a.data )[index];
		case POP_AttributeType::Int32:
			return (float)( (const int32_t*)a.data )[index];
		case POP_AttributeType::UInt32:
			return (float)( (const uint32_t*)a.data )[index];
	}
	return def;
}

float sanitizeScale( float v )
{
	float s = fabsf( v );
	return s >= kMinRenderScale ? s : kMinRenderScale;
}

} // namespace

extern "C"
{

DLLEXPORT
void FillPOPPluginInfo( POP_PluginInfo* info )
{
	if ( !info->setAPIVersion( POPCPlusPlusAPIVersion ) )
		return;
	OP_CustomOPInfo& c = info->customOPInfo;
	c.opType->setString( "Box3dinstancespop" );
	c.opLabel->setString( "Box3D Instances POP" );
	c.opIcon->setString( "INP" );
	c.authorName->setString( "lolo" );
	c.authorEmail->setString( "xxxlolx@gmail.com" );
	c.minInputs = 1;
	c.maxInputs = 1; // input 0: spawn points (one rigid body per point)
	// Existence in the physics world can't depend on something consuming the
	// output — kick-start cooking on scene load (see getGeneralInfo).
	c.cookOnStart = true;
}

DLLEXPORT
POP_CPlusPlusBase* CreatePOPInstance( const OP_NodeInfo* info, POP_Context* context )
{
	return new Box3DInstancesPOP( info, context );
}

DLLEXPORT
void DestroyPOPInstance( POP_CPlusPlusBase* instance, POP_Context* )
{
	delete (Box3DInstancesPOP*)instance;
}

} // extern "C"

Box3DInstancesPOP::Box3DInstancesPOP( const OP_NodeInfo* info, POP_Context* context )
	: myNodeInfo( info ), myContext( context ), myOpId( info->opId )
{
}

Box3DInstancesPOP::~Box3DInstancesPOP()
{
	unregisterGroup();
}

void Box3DInstancesPOP::unregisterGroup()
{
	if ( myRegistered && mySolverOpId != 0 )
	{
		SolverCore* core = Registry::find( mySolverOpId );
		if ( core != nullptr )
			core->removeGroup( myOpId );
	}
	myRegistered = false;
}

void Box3DInstancesPOP::getGeneralInfo( POP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* )
{
	// Keep Alive When Hidden (default on): cook every frame so this node's group
	// (and its heartbeat) stays alive in the physics world even when nothing
	// views the output. Off lets the scene idle when hidden — the node only cooks
	// when asked, so its bodies drop after ~4 idle frames and respawn on the next
	// cook.
	bool keepAlive = inputs->getParInt( KeepAliveName ) != 0;
	ginfo->cookEveryFrame = keepAlive;
	ginfo->cookEveryFrameIfAsked = !keepAlive;
}

Box3DInstancesPOP::Defaults Box3DInstancesPOP::readDefaults( const OP_Inputs* inputs ) const
{
	Defaults d;
	d.shape = inputs->getParInt( ShapeName );
	double sx, sy, sz;
	inputs->getParDouble3( SizeName, sx, sy, sz );
	d.sizeX = (float)sx;
	d.sizeY = (float)sy;
	d.sizeZ = (float)sz;
	// POP: use the value-returning getParDouble overload (the out-param form
	// doesn't populate here).
	d.density = (float)inputs->getParDouble( DensityName );
	d.friction = (float)inputs->getParDouble( FrictionName );
	d.restitution = (float)inputs->getParDouble( RestitutionName );
	d.type = inputs->getParInt( TypeName );
	d.bullet = inputs->getParInt( BulletName ) != 0;
	return d;
}

std::vector<SpawnBody> Box3DInstancesPOP::parseSpawnPOP( const OP_POPInput* in, const Defaults& defaults ) const
{
	std::vector<SpawnBody> defs;
	if ( in == nullptr )
		return defs;

	POP_GetBufferInfo gi;
	gi.location = POP_BufferLocation::CPU;
	OP_SmartRef<POP_Buffer> piBuf = in->getPointInfo( gi, nullptr );
	int n = piBuf ? (int)( (POP_PointInfo*)piBuf->getData( nullptr ) )->numPoints : 0;
	if ( n <= 0 )
		return defs;

	PopAttr aP = fetchPointAttr( in, "P" );
	if ( !aP )
		return defs;

	PopAttr aOrient = fetchPointAttr( in, "orient" );
	PopAttr aRot = fetchPointAttr( in, "rot" );
	PopAttr aScale = fetchPointAttr( in, "scale" );
	PopAttr aSize = fetchPointAttr( in, "size" );
	PopAttr aShape = fetchPointAttr( in, "shape" );
	PopAttr aDensity = fetchPointAttr( in, "density" );
	PopAttr aFriction = fetchPointAttr( in, "friction" );
	PopAttr aRestitution = fetchPointAttr( in, "restitution" );
	PopAttr aType = fetchPointAttr( in, "type" );
	PopAttr aAlive = fetchPointAttr( in, "alive" );
	PopAttr aBullet = fetchPointAttr( in, "bullet" );

	const PopAttr& aSc = aScale ? aScale : aSize; // scale wins, size is an alias

	defs.reserve( n );
	for ( int i = 0; i < n; ++i )
	{
		SpawnBody d;
		d.px = readComp( aP, i, 0, 0.0f );
		d.py = readComp( aP, i, 1, 0.0f );
		d.pz = readComp( aP, i, 2, 0.0f );

		if ( aOrient && aOrient.comps >= 4 )
		{
			d.qx = readComp( aOrient, i, 0, 0.0f );
			d.qy = readComp( aOrient, i, 1, 0.0f );
			d.qz = readComp( aOrient, i, 2, 0.0f );
			d.qw = readComp( aOrient, i, 3, 1.0f );
		}
		else if ( aRot )
		{
			float rx = readComp( aRot, i, 0, 0.0f );
			float ry = readComp( aRot, i, 1, 0.0f );
			float rz = readComp( aRot, i, 2, 0.0f );
			eulerXYZDegreesToQuat( rx, ry, rz, d.qx, d.qy, d.qz, d.qw );
		}

		d.shape = (int)readComp( aShape, i, 0, (float)defaults.shape );
		if ( aSc )
		{
			d.sizeX = readComp( aSc, i, 0, defaults.sizeX );
			d.sizeY = readComp( aSc, i, 1, d.sizeX );
			d.sizeZ = readComp( aSc, i, 2, d.sizeX );
		}
		else
		{
			d.sizeX = defaults.sizeX;
			d.sizeY = defaults.sizeY;
			d.sizeZ = defaults.sizeZ;
		}
		d.density = readComp( aDensity, i, 0, defaults.density );
		d.friction = readComp( aFriction, i, 0, defaults.friction );
		d.restitution = readComp( aRestitution, i, 0, defaults.restitution );

		// alive overrides type: 1 = dynamic, 0 = static (frozen).
		if ( aAlive )
			d.type = readComp( aAlive, i, 0, 1.0f ) > 0.5f ? 2 : 0;
		else
			d.type = (int)readComp( aType, i, 0, (float)defaults.type );

		d.bullet = readComp( aBullet, i, 0, defaults.bullet ? 1.0f : 0.0f ) > 0.5f;

		defs.push_back( d );
	}
	return defs;
}

void Box3DInstancesPOP::execute( POP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();

	// Reading the solver CHOP creates the cook dependency: TD steps the solver
	// before this node cooks.
	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	SolverCore* core = solverInput != nullptr ? Registry::find( solverInput->opId ) : nullptr;
	if ( core == nullptr )
	{
		myWarning = "Set the Solver parameter to a Box3D Solver CHOP.";
		unregisterGroup();
		mySolverOpId = 0;
		myLastSpawnDefs.clear();
		return;
	}
	if ( solverInput->opId != mySolverOpId )
	{
		unregisterGroup();
		mySolverOpId = solverInput->opId;
	}
	if ( myResetPending )
	{
		core->removeGroup( myOpId );
		myRegistered = false;
		myHaveLast = false;
		myLastSpawnDefs.clear();
		myPopId = 0;
		myPopCooks = -1;
		myResetPending = false;
	}

	const OP_POPInput* in = inputs->getInputPOP( 0 );
	if ( in == nullptr )
	{
		myWarning = "Connect spawn points to input 0 (one rigid body per point).";
		return;
	}

	Defaults defaults = readDefaults( inputs );
	uint32_t popId = in->opId;
	int64_t popCooks = in->totalCooks;

	bool groupChanged = !myRegistered || !myHaveLast || defaults != myLastDefaults || popId != myPopId ||
						popCooks != myPopCooks;
	if ( groupChanged )
	{
		std::vector<SpawnBody> newDefs = parseSpawnPOP( in, defaults );

		// Freeze/resume in place: when an instance only flips its body type
		// (alive/type) with the same shape & size, recreate it at its CURRENT
		// live pose instead of snapping back to its spawn point. Holds while the
		// spawn input is stable; a continuously-cooking input keeps re-driving
		// static bodies to their spawn points, which is the intended follow.
		if ( myRegistered && myHaveLast && !myLastSpawnDefs.empty() )
		{
			std::vector<BodyTransform> live( myLastSpawnDefs.size() );
			int got = core->getGroupTransforms( myOpId, live.data(), (int)live.size() );
			size_t common = newDefs.size() < (size_t)got ? newDefs.size() : (size_t)got;
			for ( size_t i = 0; i < common; ++i )
			{
				const SpawnBody& prev = myLastSpawnDefs[i];
				SpawnBody& cur = newDefs[i];
				bool sameBody = prev.shape == cur.shape && prev.sizeX == cur.sizeX && prev.sizeY == cur.sizeY &&
								prev.sizeZ == cur.sizeZ && prev.bullet == cur.bullet;
				if ( sameBody && prev.type != cur.type )
				{
					cur.px = live[i].px; cur.py = live[i].py; cur.pz = live[i].pz;
					cur.qx = live[i].qx; cur.qy = live[i].qy; cur.qz = live[i].qz; cur.qw = live[i].qw;
				}
			}
		}

		myLastSpawnDefs = std::move( newDefs );
		core->setGroup( myOpId, myLastSpawnDefs );
		myRegistered = true;
		myHaveLast = true;
		myLastDefaults = defaults;
		myPopId = popId;
		myPopCooks = popCooks;
	}
	// Keep the node path registered (heartbeat + rename/move tracking).
	core->setGroupPath( myOpId, myNodeInfo->opPath != nullptr ? myNodeInfo->opPath : "" );

	const int N = (int)myLastSpawnDefs.size();
	const bool extra = inputs->getParInt( ExtrachansName ) != 0;

	// ---- live states (the solver stepped during its own cook) ----
	std::vector<BodyState> states( N > 0 ? N : 1 );
	int written = N > 0 ? core->getGroupStates( myOpId, states.data(), N ) : 0;

	// Extra per-instance visualization data (angular velocity, speed, awake and
	// live contacts) so a material can map motion/impact to color.
	std::vector<BodyContactState> contacts;
	int cwritten = 0;
	if ( extra && N > 0 )
	{
		contacts.resize( N );
		cwritten = core->getGroupContactStates( myOpId, contacts.data(), N );
	}

	// ---- output buffers: P, rot (TD Euler XYZ deg), scale, v ----
	auto makeVec3 = [&]() -> OP_SmartRef<POP_Buffer> {
		POP_BufferInfo bi;
		bi.size = (uint64_t)( N > 0 ? N : 1 ) * 3 * sizeof( float );
		bi.location = POP_BufferLocation::CPU;
		return myContext->createBuffer( bi, nullptr );
	};
	auto makeScalar = [&]() -> OP_SmartRef<POP_Buffer> {
		POP_BufferInfo bi;
		bi.size = (uint64_t)( N > 0 ? N : 1 ) * sizeof( float );
		bi.location = POP_BufferLocation::CPU;
		return myContext->createBuffer( bi, nullptr );
	};

	OP_SmartRef<POP_Buffer> outP = makeVec3();
	OP_SmartRef<POP_Buffer> outRot = makeVec3();
	OP_SmartRef<POP_Buffer> outScale = makeVec3();
	OP_SmartRef<POP_Buffer> outV = makeVec3();
	if ( !outP || !outRot || !outScale || !outV )
		return;

	float* dP = (float*)outP->getData( nullptr );
	float* dRot = (float*)outRot->getData( nullptr );
	float* dScale = (float*)outScale->getData( nullptr );
	float* dV = (float*)outV->getData( nullptr );

	OP_SmartRef<POP_Buffer> outW, outSpeed, outAwake, outTouch, outImpulse, outHit;
	float *dW = nullptr, *dSpeed = nullptr, *dAwake = nullptr, *dTouch = nullptr, *dImp = nullptr, *dHit = nullptr;
	bool haveExtra = false;
	if ( extra )
	{
		outW = makeVec3();
		outSpeed = makeScalar();
		outAwake = makeScalar();
		outTouch = makeScalar();
		outImpulse = makeScalar();
		outHit = makeScalar();
		if ( outW && outSpeed && outAwake && outTouch && outImpulse && outHit )
		{
			dW = (float*)outW->getData( nullptr );
			dSpeed = (float*)outSpeed->getData( nullptr );
			dAwake = (float*)outAwake->getData( nullptr );
			dTouch = (float*)outTouch->getData( nullptr );
			dImp = (float*)outImpulse->getData( nullptr );
			dHit = (float*)outHit->getData( nullptr );
			haveExtra = true;
		}
	}

	for ( int i = 0; i < N; ++i )
	{
		float px = 0, py = 0, pz = 0, rx = 0, ry = 0, rz = 0, vx = 0, vy = 0, vz = 0;
		if ( i < written )
		{
			const BodyState& s = states[i];
			px = s.px; py = s.py; pz = s.pz;
			quatToEulerXYZDegrees( s.qx, s.qy, s.qz, s.qw, rx, ry, rz );
			vx = s.vx; vy = s.vy; vz = s.vz;
		}
		else if ( i < (int)myLastSpawnDefs.size() )
		{
			// Fallback to the spawn pose until the world is built (1 frame).
			const SpawnBody& b = myLastSpawnDefs[i];
			px = b.px; py = b.py; pz = b.pz;
			quatToEulerXYZDegrees( b.qx, b.qy, b.qz, b.qw, rx, ry, rz );
		}
		dP[i * 3 + 0] = px; dP[i * 3 + 1] = py; dP[i * 3 + 2] = pz;
		dRot[i * 3 + 0] = rx; dRot[i * 3 + 1] = ry; dRot[i * 3 + 2] = rz;
		dV[i * 3 + 0] = vx; dV[i * 3 + 1] = vy; dV[i * 3 + 2] = vz;

		// scale from the spawn size, mirrored to match the collision shape
		// (sphere is uniform, capsule is round in XZ).
		float sx = 1, sy = 1, sz = 1;
		if ( i < (int)myLastSpawnDefs.size() )
		{
			const SpawnBody& b = myLastSpawnDefs[i];
			sx = sanitizeScale( b.sizeX );
			sy = sanitizeScale( b.sizeY );
			sz = sanitizeScale( b.sizeZ );
			if ( b.shape == 1 ) { sy = sx; sz = sx; }
			else if ( b.shape == 2 ) { sz = sx; }
		}
		dScale[i * 3 + 0] = sx; dScale[i * 3 + 1] = sy; dScale[i * 3 + 2] = sz;

		if ( haveExtra )
		{
			float wx = 0, wy = 0, wz = 0, aw = 0;
			if ( i < written )
			{
				const BodyState& s = states[i];
				wx = s.wx; wy = s.wy; wz = s.wz; aw = s.awake;
			}
			dW[i * 3 + 0] = wx; dW[i * 3 + 1] = wy; dW[i * 3 + 2] = wz;
			dSpeed[i] = sqrtf( vx * vx + vy * vy + vz * vz );
			dAwake[i] = aw;
			float tch = 0, imp = 0, hit = 0;
			if ( i < cwritten )
			{
				tch = contacts[i].touching; imp = contacts[i].impulse; hit = contacts[i].hitSpeed;
			}
			dTouch[i] = tch; dImp[i] = imp; dHit[i] = hit;
		}
	}

	// ---- point info ----
	POP_InfoBuffers infoBufs;
	{
		POP_BufferInfo pib;
		pib.usage = POP_BufferUsage::PointInfoBuffer;
		pib.size = sizeof( POP_PointInfo );
		pib.location = POP_BufferLocation::CPU;
		pib.mode = POP_BufferMode::ReadWrite;
		infoBufs.pointInfo = myContext->createBuffer( pib, nullptr );
		if ( infoBufs.pointInfo )
		{
			void* pd = infoBufs.pointInfo->getData( nullptr );
			memset( pd, 0, sizeof( POP_PointInfo ) );
			( (POP_PointInfo*)pd )->numPoints = (uint32_t)( N > 0 ? N : 0 );
		}
	}

	// ---- register outputs ----
	POP_SetBufferInfo sinfo;
	auto emit = [&]( OP_SmartRef<POP_Buffer>& buf, const char* name, bool direction ) {
		POP_AttributeInfo ai;
		ai.name = name;
		ai.numComponents = 3;
		ai.type = POP_AttributeType::Float;
		ai.attribClass = POP_AttributeClass::Point;
		if ( direction )
			ai.qualifier = POP_AttributeQualifier::Direction;
		output->setAttribute( &buf, ai, sinfo, nullptr );
	};
	auto emitScalar = [&]( OP_SmartRef<POP_Buffer>& buf, const char* name ) {
		POP_AttributeInfo ai;
		ai.name = name;
		ai.numComponents = 1;
		ai.type = POP_AttributeType::Float;
		ai.attribClass = POP_AttributeClass::Point;
		output->setAttribute( &buf, ai, sinfo, nullptr );
	};
	if ( N > 0 )
	{
		emit( outP, "P", false );
		emit( outRot, "rot", false );
		emit( outScale, "scale", false );
		emit( outV, "v", true );
		if ( haveExtra )
		{
			emit( outW, "w", true );
			emitScalar( outSpeed, "speed" );
			emitScalar( outAwake, "awake" );
			emitScalar( outTouch, "touching" );
			emitScalar( outImpulse, "impulse" );
			emitScalar( outHit, "hitspeed" );
		}
	}
	output->setInfoBuffers( &infoBufs, sinfo, nullptr );
}

int32_t Box3DInstancesPOP::getNumInfoCHOPChans( void* )
{
	return 1;
}

void Box3DInstancesPOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	if ( index == 0 )
	{
		chan->name->setString( "body_count" );
		SolverCore* core = mySolverOpId != 0 ? Registry::find( mySolverOpId ) : nullptr;
		chan->value = core != nullptr ? (float)core->getGroupBodyCount( myOpId ) : 0.0f;
	}
}

void Box3DInstancesPOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
		warning->setString( myWarning.c_str() );
}

void Box3DInstancesPOP::pulsePressed( const char* name, void* )
{
	if ( std::strcmp( name, ResetName ) == 0 )
		myResetPending = true;
}

void Box3DInstancesPOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Instances";
		p.defaultValue = "Box3dsolver1";
		manager->appendCHOP( p );
	}
	{
		OP_NumericParameter p;
		p.name = ResetName;
		p.label = "Reset";
		p.page = "Instances";
		manager->appendPulse( p );
	}
	{
		OP_StringParameter p;
		p.name = ShapeName;
		p.label = "Default Shape";
		p.page = "Instances";
		p.defaultValue = "box";
		const char* names[] = { "box", "sphere", "capsule" };
		const char* labels[] = { "Box", "Sphere", "Capsule" };
		manager->appendMenu( p, 3, names, labels );
	}
	{
		OP_NumericParameter p;
		p.name = SizeName;
		p.label = "Default Size";
		p.page = "Instances";
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
	auto flt = [&]( const char* nm, const char* lb, double def, double hi ) {
		OP_NumericParameter p;
		p.name = nm;
		p.label = lb;
		p.page = "Instances";
		p.defaultValues[0] = def;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = hi;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	};
	flt( DensityName, "Default Density", 1.0, 10.0 );
	flt( FrictionName, "Default Friction", 0.6, 1.0 );
	flt( RestitutionName, "Default Restitution", 0.0, 1.0 );
	{
		OP_StringParameter p;
		p.name = TypeName;
		p.label = "Default Type";
		p.page = "Instances";
		p.defaultValue = "dynamic";
		const char* names[] = { "static", "kinematic", "dynamic" };
		const char* labels[] = { "Static", "Kinematic", "Dynamic" };
		manager->appendMenu( p, 3, names, labels );
	}
	{
		OP_NumericParameter p;
		p.name = BulletName;
		p.label = "CCD (Bullet)";
		p.page = "Instances";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}
	{
		// On: cook every frame so these bodies live in the world even when hidden.
		// Off: only cook when viewed/used, letting the scene idle (bodies drop
		// after ~4 idle frames and respawn on the next cook).
		OP_NumericParameter p;
		p.name = KeepAliveName;
		p.label = "Keep Alive When Hidden";
		p.page = "Instances";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}
	{
		// Extra per-point attributes for visualization: w (angular velocity),
		// speed (|v|), awake, touching, impulse, hitspeed. Map any to color.
		OP_NumericParameter p;
		p.name = ExtrachansName;
		p.label = "Extra Channels";
		p.page = "Instances";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}
}

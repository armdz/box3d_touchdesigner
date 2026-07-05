// Shared TD-side helpers for the Box3D plugins (header-only).
// Used by both Box3DSolverCHOP and Box3DBodiesCHOP. Only box3d INLINE header
// math may be used here — box3d link-level functions live inside Box3DCore.dll
// and are not exported.

#pragma once

#include "Box3DTDCore.h"
#include "CHOP_CPlusPlusBase.h"
#include "CPlusPlus_Common.h"

#include "box3d/math_functions.h"

#include <cmath>
#include <vector>

namespace tdb3
{

constexpr int kNumOutputChannels = 6;
inline const char* kOutputChannelNames[kNumOutputChannels] = { "tx", "ty", "tz", "rx", "ry", "rz" };

// Shared parameter names for the body-defaults block (TD convention: capital
// first letter, lowercase rest)
constexpr char SpawnsopParName[] = "Spawnsop";
constexpr char ShapeParName[] = "Shape";
constexpr char SizeParName[] = "Size";
constexpr char DensityParName[] = "Density";
constexpr char FrictionParName[] = "Friction";
constexpr char RestitutionParName[] = "Restitution";
constexpr char TypeParName[] = "Type";

struct SpawnDefaults
{
	int shape = 0;
	float sizeX = 1.0f, sizeY = 1.0f, sizeZ = 1.0f;
	float density = 1.0f;
	float friction = 0.6f;
	float restitution = 0.0f;
	int type = 2;

	bool operator!=( const SpawnDefaults& o ) const
	{
		return shape != o.shape || sizeX != o.sizeX || sizeY != o.sizeY || sizeZ != o.sizeZ || density != o.density ||
			   friction != o.friction || restitution != o.restitution || type != o.type;
	}
};

inline SpawnDefaults readSpawnDefaults( const TD::OP_Inputs* inputs )
{
	SpawnDefaults d;
	d.shape = inputs->getParInt( ShapeParName );

	double sx, sy, sz;
	inputs->getParDouble3( SizeParName, sx, sy, sz );
	d.sizeX = (float)sx;
	d.sizeY = (float)sy;
	d.sizeZ = (float)sz;

	d.density = (float)inputs->getParDouble( DensityParName );
	d.friction = (float)inputs->getParDouble( FrictionParName );
	d.restitution = (float)inputs->getParDouble( RestitutionParName );
	d.type = inputs->getParInt( TypeParName );
	return d;
}

// Appends the shared body parameters (Spawn SOP + defaults) to the given page.
inline void appendBodyParameters( TD::OP_ParameterManager* manager, const char* page )
{
	{
		TD::OP_StringParameter p;
		p.name = SpawnsopParName;
		p.label = "Spawn SOP";
		p.page = page;
		manager->appendSOP( p );
	}

	{
		TD::OP_StringParameter p;
		p.name = ShapeParName;
		p.label = "Default Shape";
		p.page = page;
		p.defaultValue = "box";
		const char* names[] = { "box", "sphere", "capsule" };
		const char* labels[] = { "Box", "Sphere", "Capsule" };
		manager->appendMenu( p, 3, names, labels );
	}

	{
		TD::OP_NumericParameter p;
		p.name = SizeParName;
		p.label = "Default Size";
		p.page = page;
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
		TD::OP_NumericParameter p;
		p.name = DensityParName;
		p.label = "Default Density";
		p.page = page;
		p.defaultValues[0] = 1.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 10.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		TD::OP_NumericParameter p;
		p.name = FrictionParName;
		p.label = "Default Friction";
		p.page = page;
		p.defaultValues[0] = 0.6;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 1.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		TD::OP_NumericParameter p;
		p.name = RestitutionParName;
		p.label = "Default Restitution";
		p.page = page;
		p.defaultValues[0] = 0.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 1.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		TD::OP_StringParameter p;
		p.name = TypeParName;
		p.label = "Default Type";
		p.page = page;
		p.defaultValue = "dynamic";
		const char* names[] = { "static", "kinematic", "dynamic" };
		const char* labels[] = { "Static", "Kinematic", "Dynamic" };
		manager->appendMenu( p, 3, names, labels );
	}
}

// Read one component of an optional per-point attribute, falling back to a
// default when the attribute or component is missing.
inline float readPointAttr( const TD::SOP_CustomAttribData* a, int point, int comp, float def )
{
	if ( a == nullptr || comp >= a->numComponents )
	{
		return def;
	}

	int index = point * a->numComponents + comp;
	if ( a->attribType == TD::AttribType::Float && a->floatData != nullptr )
	{
		return a->floatData[index];
	}
	if ( a->attribType == TD::AttribType::Int && a->intData != nullptr )
	{
		return (float)a->intData[index];
	}
	return def;
}

// Converts Euler XYZ degrees (TD-style RX/RY/RZ) to quaternion x y z w.
inline void eulerXYZDegreesToQuat( float rxDeg, float ryDeg, float rzDeg, float& qx, float& qy, float& qz, float& qw )
{
	constexpr float kDegToRad = 0.01745329251994329577f;
	float hx = 0.5f * rxDeg * kDegToRad;
	float hy = 0.5f * ryDeg * kDegToRad;
	float hz = 0.5f * rzDeg * kDegToRad;

	float cx = cosf( hx );
	float sx = sinf( hx );
	float cy = cosf( hy );
	float sy = sinf( hy );
	float cz = cosf( hz );
	float sz = sinf( hz );

	qw = cx * cy * cz + sx * sy * sz;
	qx = sx * cy * cz - cx * sy * sz;
	qy = cx * sy * cz + sx * cy * sz;
	qz = cx * cy * sz - sx * sy * cz;
}

// Each SOP point becomes one SpawnBody. Per-point attributes (shape, size,
// density, friction, restitution, type, orient) override the defaults.
// See PLAN.md for the attribute contract.
inline std::vector<SpawnBody> parseSpawnSop( const TD::OP_SOPInput* sop, const SpawnDefaults& defaults )
{
	std::vector<SpawnBody> defs;
	if ( sop == nullptr )
	{
		return defs;
	}

	int pointCount = sop->getNumPoints();
	const TD::Position* points = sop->getPointPositions();

	const TD::SOP_CustomAttribData* shapeAttr = sop->getCustomAttribute( "shape" );
	const TD::SOP_CustomAttribData* sizeAttr = sop->getCustomAttribute( "size" );
	const TD::SOP_CustomAttribData* size0Attr = sop->getCustomAttribute( "size0" );
	const TD::SOP_CustomAttribData* size1Attr = sop->getCustomAttribute( "size1" );
	const TD::SOP_CustomAttribData* size2Attr = sop->getCustomAttribute( "size2" );
	const TD::SOP_CustomAttribData* sizeXAttr = sop->getCustomAttribute( "sizex" );
	const TD::SOP_CustomAttribData* sizeYAttr = sop->getCustomAttribute( "sizey" );
	const TD::SOP_CustomAttribData* sizeZAttr = sop->getCustomAttribute( "sizez" );
	const TD::SOP_CustomAttribData* sxAttr = sop->getCustomAttribute( "sx" );
	const TD::SOP_CustomAttribData* syAttr = sop->getCustomAttribute( "sy" );
	const TD::SOP_CustomAttribData* szAttr = sop->getCustomAttribute( "sz" );
	const TD::SOP_CustomAttribData* densityAttr = sop->getCustomAttribute( "density" );
	const TD::SOP_CustomAttribData* frictionAttr = sop->getCustomAttribute( "friction" );
	const TD::SOP_CustomAttribData* restitutionAttr = sop->getCustomAttribute( "restitution" );
	const TD::SOP_CustomAttribData* typeAttr = sop->getCustomAttribute( "type" );
	const TD::SOP_CustomAttribData* orientAttr = sop->getCustomAttribute( "orient" );
	const TD::SOP_CustomAttribData* rxAttr = sop->getCustomAttribute( "rx" );
	const TD::SOP_CustomAttribData* ryAttr = sop->getCustomAttribute( "ry" );
	const TD::SOP_CustomAttribData* rzAttr = sop->getCustomAttribute( "rz" );

	const bool hasSplitSize = size0Attr != nullptr || size1Attr != nullptr || size2Attr != nullptr ||
							 sizeXAttr != nullptr || sizeYAttr != nullptr || sizeZAttr != nullptr ||
							 sxAttr != nullptr || syAttr != nullptr || szAttr != nullptr;

	defs.reserve( pointCount );
	for ( int i = 0; i < pointCount; ++i )
	{
		SpawnBody d;
		d.px = points[i].x;
		d.py = points[i].y;
		d.pz = points[i].z;

		if ( orientAttr != nullptr && orientAttr->numComponents >= 4 )
		{
			d.qx = readPointAttr( orientAttr, i, 0, 0.0f );
			d.qy = readPointAttr( orientAttr, i, 1, 0.0f );
			d.qz = readPointAttr( orientAttr, i, 2, 0.0f );
			d.qw = readPointAttr( orientAttr, i, 3, 1.0f );
		}
		else if ( rxAttr != nullptr || ryAttr != nullptr || rzAttr != nullptr )
		{
			float rx = readPointAttr( rxAttr, i, 0, 0.0f );
			float ry = readPointAttr( ryAttr, i, 0, 0.0f );
			float rz = readPointAttr( rzAttr, i, 0, 0.0f );
			eulerXYZDegreesToQuat( rx, ry, rz, d.qx, d.qy, d.qz, d.qw );
		}

		d.shape = (int)readPointAttr( shapeAttr, i, 0, (float)defaults.shape );
		if ( sizeAttr != nullptr )
		{
			d.sizeX = readPointAttr( sizeAttr, i, 0, defaults.sizeX );
			d.sizeY = readPointAttr( sizeAttr, i, 1, d.sizeX );
			d.sizeZ = readPointAttr( sizeAttr, i, 2, d.sizeX );
		}
		else if ( hasSplitSize )
		{
			d.sizeX = readPointAttr( size0Attr, i, 0,
									readPointAttr( sizeXAttr, i, 0, readPointAttr( sxAttr, i, 0, defaults.sizeX ) ) );
			d.sizeY = readPointAttr( size1Attr, i, 0,
									readPointAttr( sizeYAttr, i, 0, readPointAttr( syAttr, i, 0, d.sizeX ) ) );
			d.sizeZ = readPointAttr( size2Attr, i, 0,
									readPointAttr( sizeZAttr, i, 0, readPointAttr( szAttr, i, 0, d.sizeX ) ) );
		}
		else
		{
			d.sizeX = defaults.sizeX;
			d.sizeY = defaults.sizeY;
			d.sizeZ = defaults.sizeZ;
		}
		d.density = readPointAttr( densityAttr, i, 0, defaults.density );
		d.friction = readPointAttr( frictionAttr, i, 0, defaults.friction );
		d.restitution = readPointAttr( restitutionAttr, i, 0, defaults.restitution );
		d.type = (int)readPointAttr( typeAttr, i, 0, (float)defaults.type );

		defs.push_back( d );
	}

	return defs;
}

// Euler angles in degrees matching TD's default rotate order (XYZ: rotate X,
// then Y, then Z), which is what Geometry COMP instancing expects in RX/RY/RZ.
// That composition is R = Rz*Ry*Rx with column vectors.
inline void quatToEulerXYZDegrees( float qx, float qy, float qz, float qw, float* rx, float* ry, float* rz )
{
	constexpr float kRadToDeg = 57.295779513082320877f;

	b3Quat q;
	q.v.x = qx;
	q.v.y = qy;
	q.v.z = qz;
	q.s = qw;
	b3Matrix3 m = b3MakeMatrixFromQuat( q );

	float sinPitch = -m.cx.z;
	if ( sinPitch >= 0.99999f )
	{
		// gimbal lock looking "up": only rx+rz is defined, put it all in rx
		*rx = atan2f( m.cy.x, m.cy.y ) * kRadToDeg;
		*ry = 90.0f;
		*rz = 0.0f;
	}
	else if ( sinPitch <= -0.99999f )
	{
		*rx = atan2f( -m.cy.x, m.cy.y ) * kRadToDeg;
		*ry = -90.0f;
		*rz = 0.0f;
	}
	else
	{
		*rx = atan2f( m.cy.z, m.cz.z ) * kRadToDeg;
		*ry = asinf( sinPitch ) * kRadToDeg;
		*rz = atan2f( m.cx.y, m.cx.x ) * kRadToDeg;
	}
}

// Writes a group's transforms into the 6 output channels, zero-filling any
// samples beyond the live body count so stale data never leaks.
inline void writeTransformChannels( TD::CHOP_Output* output, SolverCore* core, uint32_t groupKey )
{
	int written = 0;
	if ( core != nullptr && output->numSamples > 0 )
	{
		std::vector<BodyTransform> transforms( output->numSamples );
		written = core->getGroupTransforms( groupKey, transforms.data(), output->numSamples );

		for ( int i = 0; i < written; ++i )
		{
			const BodyTransform& t = transforms[i];
			float rx, ry, rz;
			quatToEulerXYZDegrees( t.qx, t.qy, t.qz, t.qw, &rx, &ry, &rz );

			output->channels[0][i] = t.px;
			output->channels[1][i] = t.py;
			output->channels[2][i] = t.pz;
			output->channels[3][i] = rx;
			output->channels[4][i] = ry;
			output->channels[5][i] = rz;
		}
	}

	for ( int i = written; i < output->numSamples; ++i )
	{
		for ( int c = 0; c < kNumOutputChannels; ++c )
		{
			output->channels[c][i] = 0.0f;
		}
	}
}

} // namespace tdb3

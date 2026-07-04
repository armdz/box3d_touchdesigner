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

struct SpawnDefaults
{
	int shape = 0;
	float sizeX = 1.0f, sizeY = 1.0f, sizeZ = 1.0f;
	float density = 1.0f;
	float friction = 0.6f;

	bool operator!=( const SpawnDefaults& o ) const
	{
		return shape != o.shape || sizeX != o.sizeX || sizeY != o.sizeY || sizeZ != o.sizeZ || density != o.density ||
			   friction != o.friction;
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
	const TD::SOP_CustomAttribData* densityAttr = sop->getCustomAttribute( "density" );
	const TD::SOP_CustomAttribData* frictionAttr = sop->getCustomAttribute( "friction" );
	const TD::SOP_CustomAttribData* restitutionAttr = sop->getCustomAttribute( "restitution" );
	const TD::SOP_CustomAttribData* typeAttr = sop->getCustomAttribute( "type" );
	const TD::SOP_CustomAttribData* orientAttr = sop->getCustomAttribute( "orient" );

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

		d.shape = (int)readPointAttr( shapeAttr, i, 0, (float)defaults.shape );
		d.sizeX = readPointAttr( sizeAttr, i, 0, defaults.sizeX );
		d.sizeY = readPointAttr( sizeAttr, i, 1, sizeAttr != nullptr ? d.sizeX : defaults.sizeY );
		d.sizeZ = readPointAttr( sizeAttr, i, 2, sizeAttr != nullptr ? d.sizeX : defaults.sizeZ );
		d.density = readPointAttr( densityAttr, i, 0, defaults.density );
		d.friction = readPointAttr( frictionAttr, i, 0, defaults.friction );
		d.restitution = readPointAttr( restitutionAttr, i, 0, 0.0f );
		d.type = (int)readPointAttr( typeAttr, i, 0, 2.0f );

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

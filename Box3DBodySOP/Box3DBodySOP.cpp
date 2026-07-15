#include "Box3DBodySOP.h"

#include "TDB3Common.h"
#include "TDB3Mesh.h"

#include "box3d/math_functions.h"

#include <cmath>
#include <cstring>

using namespace tdb3;

namespace
{

constexpr char SolverName[] = "Solver";
constexpr char ResetName[] = "Reset";
constexpr char ShapeName[] = "Shape";
constexpr char SizeName[] = "Size";
constexpr char PositionName[] = "Position";
constexpr char JointName[] = "Joint";
constexpr char JointpivotName[] = "Jointpivot";
constexpr char ShowjointpivotName[] = "Showjointpivot";
constexpr char JointenabledAttrName[] = "joint_enabled";
constexpr char JointpivotAttrName[] = "joint_pivot";
constexpr char JointpivotspaceAttrName[] = "joint_pivot_space";
constexpr char JointrefAttrName[] = "joint_ref";
constexpr char JointrefwAttrName[] = "joint_ref_w";
constexpr char TypeName[] = "Type";
constexpr char BulletccdName[] = "Bulletccd";
constexpr char WallthicknessName[] = "Wallthickness";
constexpr char OpentopName[] = "Opentop";
constexpr char ShowcollisionName[] = "Showcollision";
constexpr char DensityName[] = "Density";
constexpr char FrictionName[] = "Friction";
constexpr char RestitutionName[] = "Restitution";

// Menu index (input hull, box, sphere, capsule, mesh, compound) → core SpawnBody::shape
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
		case 4:
			return 4; // exact triangle mesh (static/kinematic)
		case 5:
			return 5; // compound of hulls (concave OK, dynamic OK)
		case 6:
			return 6; // hollow box / container (collision faces inward)
		default:
			return 3; // input hull
	}
}

// Union-find over point indices; primitives weld their points into islands.
int32_t findRoot( std::vector<int32_t>& parent, int32_t i )
{
	while ( parent[i] != i )
	{
		parent[i] = parent[parent[i]]; // path halving
		i = parent[i];
	}
	return i;
}

void unionRoots( std::vector<int32_t>& parent, int32_t a, int32_t b )
{
	int32_t ra = findRoot( parent, a );
	int32_t rb = findRoot( parent, b );
	if ( ra != rb )
	{
		parent[rb] = ra;
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

b3Matrix3 rotationFromQuat( float qx, float qy, float qz, float qw )
{
	b3Quat q;
	q.v.x = qx;
	q.v.y = qy;
	q.v.z = qz;
	q.s = qw;
	return b3MakeMatrixFromQuat( q );
}

b3Matrix3 transposeMatrix( const b3Matrix3& m )
{
	b3Matrix3 t;
	t.cx = b3Vec3{ m.cx.x, m.cy.x, m.cz.x };
	t.cy = b3Vec3{ m.cx.y, m.cy.y, m.cz.y };
	t.cz = b3Vec3{ m.cx.z, m.cy.z, m.cz.z };
	return t;
}

b3Matrix3 multiplyMatrix( const b3Matrix3& a, const b3Matrix3& b )
{
	b3Matrix3 c;
	c.cx = b3Vec3{ a.cx.x * b.cx.x + a.cy.x * b.cx.y + a.cz.x * b.cx.z,
					  a.cx.y * b.cx.x + a.cy.y * b.cx.y + a.cz.y * b.cx.z,
					  a.cx.z * b.cx.x + a.cy.z * b.cx.y + a.cz.z * b.cx.z };
	c.cy = b3Vec3{ a.cx.x * b.cy.x + a.cy.x * b.cy.y + a.cz.x * b.cy.z,
					  a.cx.y * b.cy.x + a.cy.y * b.cy.y + a.cz.y * b.cy.z,
					  a.cx.z * b.cy.x + a.cy.z * b.cy.y + a.cz.z * b.cy.z };
	c.cz = b3Vec3{ a.cx.x * b.cz.x + a.cy.x * b.cz.y + a.cz.x * b.cz.z,
					  a.cx.y * b.cz.x + a.cy.y * b.cz.y + a.cz.y * b.cz.z,
					  a.cx.z * b.cz.x + a.cy.z * b.cz.y + a.cz.z * b.cz.z };
	return c;
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

struct FrameBasis
{
	float x[3];
	float y[3];
	float z[3];
};

inline void vecSub( const Position& a, const Position& b, float out[3] )
{
	out[0] = a.x - b.x;
	out[1] = a.y - b.y;
	out[2] = a.z - b.z;
}

inline float vecDot( const float a[3], const float b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void vecCross( const float a[3], const float b[3], float out[3] )
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float vecLenSq( const float v[3] )
{
	return vecDot( v, v );
}

bool normalizeVec( float v[3] )
{
	float lenSq = vecLenSq( v );
	if ( lenSq < 1e-12f )
	{
		return false;
	}
	float inv = 1.0f / sqrtf( lenSq );
	v[0] *= inv;
	v[1] *= inv;
	v[2] *= inv;
	return true;
}

bool chooseAnchorIndices( const Position* points, int pointCount, float cx, float cy, float cz, int out[3] )
{
	if ( points == nullptr || pointCount < 3 )
	{
		return false;
	}

	int i0 = 0;
	float best0 = -1.0f;
	for ( int i = 0; i < pointCount; ++i )
	{
		float dx = points[i].x - cx;
		float dy = points[i].y - cy;
		float dz = points[i].z - cz;
		float d2 = dx * dx + dy * dy + dz * dz;
		if ( d2 > best0 )
		{
			best0 = d2;
			i0 = i;
		}
	}

	int i1 = i0 == 0 ? 1 : 0;
	float best1 = -1.0f;
	for ( int i = 0; i < pointCount; ++i )
	{
		if ( i == i0 )
		{
			continue;
		}
		float dx = points[i].x - points[i0].x;
		float dy = points[i].y - points[i0].y;
		float dz = points[i].z - points[i0].z;
		float d2 = dx * dx + dy * dy + dz * dz;
		if ( d2 > best1 )
		{
			best1 = d2;
			i1 = i;
		}
	}

	float e01[3] = { points[i1].x - points[i0].x, points[i1].y - points[i0].y, points[i1].z - points[i0].z };

	int i2 = -1;
	float bestAreaSq = -1.0f;
	for ( int i = 0; i < pointCount; ++i )
	{
		if ( i == i0 || i == i1 )
		{
			continue;
		}
		float e02[3] = { points[i].x - points[i0].x, points[i].y - points[i0].y, points[i].z - points[i0].z };
		float c[3];
		vecCross( e01, e02, c );
		float areaSq = vecLenSq( c );
		if ( areaSq > bestAreaSq )
		{
			bestAreaSq = areaSq;
			i2 = i;
		}
	}

	if ( i2 < 0 || bestAreaSq < 1e-12f )
	{
		return false;
	}

	out[0] = i0;
	out[1] = i1;
	out[2] = i2;
	return true;
}

void addJointPreviewMarker( SOP_Output* output, bool enabled, bool showPivot, float pivotX, float pivotY, float pivotZ,
								  float sizeX, float sizeY, float sizeZ, const BodyTransform& t, std::vector<Position>& bounds )
{
	if ( !enabled || !showPivot )
	{
		return;
	}

	b3Matrix3 r = rotationFromTransform( t );
	float span = 0.12f * ( sizeX > sizeY ? ( sizeX > sizeZ ? sizeX : sizeZ ) : ( sizeY > sizeZ ? sizeY : sizeZ ) );
	if ( span < 0.08f )
	{
		span = 0.08f;
	}

	Position c = applyTransform( r, t, pivotX, pivotY, pivotZ );
	Position px = applyTransform( r, t, pivotX + span, pivotY, pivotZ );
	Position py = applyTransform( r, t, pivotX, pivotY + span, pivotZ );
	Position pz = applyTransform( r, t, pivotX, pivotY, pivotZ + span );

	int32_t ic = output->addPoint( c );
	int32_t ix = output->addPoint( px );
	int32_t iy = output->addPoint( py );
	int32_t iz = output->addPoint( pz );
	const int32_t lx[2] = { ic, ix };
	const int32_t ly[2] = { ic, iy };
	const int32_t lz[2] = { ic, iz };
	output->addLine( lx, 2 );
	output->addLine( ly, 2 );
	output->addLine( lz, 2 );

	bounds.push_back( c );
	bounds.push_back( px );
	bounds.push_back( py );
	bounds.push_back( pz );
}

bool buildFrameFromAnchors( const Position* points, int pointCount, const int anchors[3], FrameBasis& out )
{
	if ( points == nullptr || pointCount < 3 )
	{
		return false;
	}
	if ( anchors[0] < 0 || anchors[0] >= pointCount || anchors[1] < 0 || anchors[1] >= pointCount || anchors[2] < 0 ||
		 anchors[2] >= pointCount )
	{
		return false;
	}

	const Position& p0 = points[anchors[0]];
	const Position& p1 = points[anchors[1]];
	const Position& p2 = points[anchors[2]];

	out.x[0] = p1.x - p0.x;
	out.x[1] = p1.y - p0.y;
	out.x[2] = p1.z - p0.z;
	if ( !normalizeVec( out.x ) )
	{
		return false;
	}

	float t[3] = { p2.x - p0.x, p2.y - p0.y, p2.z - p0.z };
	vecCross( out.x, t, out.z );
	if ( !normalizeVec( out.z ) )
	{
		return false;
	}

	vecCross( out.z, out.x, out.y );
	return normalizeVec( out.y );
}

void quatFromFrame( const FrameBasis& f, float& qx, float& qy, float& qz, float& qw )
{
	float m00 = f.x[0], m01 = f.y[0], m02 = f.z[0];
	float m10 = f.x[1], m11 = f.y[1], m12 = f.z[1];
	float m20 = f.x[2], m21 = f.y[2], m22 = f.z[2];

	float trace = m00 + m11 + m22;
	if ( trace > 0.0f )
	{
		float s = sqrtf( trace + 1.0f ) * 2.0f;
		qw = 0.25f * s;
		qx = ( m21 - m12 ) / s;
		qy = ( m02 - m20 ) / s;
		qz = ( m10 - m01 ) / s;
	}
	else if ( m00 > m11 && m00 > m22 )
	{
		float s = sqrtf( 1.0f + m00 - m11 - m22 ) * 2.0f;
		qw = ( m21 - m12 ) / s;
		qx = 0.25f * s;
		qy = ( m01 + m10 ) / s;
		qz = ( m02 + m20 ) / s;
	}
	else if ( m11 > m22 )
	{
		float s = sqrtf( 1.0f + m11 - m00 - m22 ) * 2.0f;
		qw = ( m02 - m20 ) / s;
		qx = ( m01 + m10 ) / s;
		qy = 0.25f * s;
		qz = ( m12 + m21 ) / s;
	}
	else
	{
		float s = sqrtf( 1.0f + m22 - m00 - m11 ) * 2.0f;
		qw = ( m10 - m01 ) / s;
		qx = ( m02 + m20 ) / s;
		qy = ( m12 + m21 ) / s;
		qz = 0.25f * s;
	}
}

// TD object matrices are [row][col] with column-vector convention: rotation
// axes in the columns, translation in [0..2][3]. Scale is stripped by
// normalizing the axes (rigid bodies cannot scale).
void poseFromObjectTransform( const double m[4][4], float& px, float& py, float& pz, float& qx, float& qy, float& qz,
							  float& qw )
{
	px = (float)m[0][3];
	py = (float)m[1][3];
	pz = (float)m[2][3];

	FrameBasis f;
	f.x[0] = (float)m[0][0];
	f.x[1] = (float)m[1][0];
	f.x[2] = (float)m[2][0];
	f.y[0] = (float)m[0][1];
	f.y[1] = (float)m[1][1];
	f.y[2] = (float)m[2][1];
	f.z[0] = (float)m[0][2];
	f.z[1] = (float)m[1][2];
	f.z[2] = (float)m[2][2];

	if ( !normalizeVec( f.x ) || !normalizeVec( f.y ) || !normalizeVec( f.z ) )
	{
		qx = qy = qz = 0.0f;
		qw = 1.0f;
		return;
	}

	quatFromFrame( f, qx, qy, qz, qw );
}

float edgeLength( const Position& a, const Position& b )
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrtf( dx * dx + dy * dy + dz * dz );
}

// Bounding-box diagonal of a point cloud; 0 for empty/degenerate input.
float cloudDiagonal( const Position* points, int count )
{
	if ( points == nullptr || count <= 0 )
	{
		return 0.0f;
	}
	float minv[3] = { points[0].x, points[0].y, points[0].z };
	float maxv[3] = { points[0].x, points[0].y, points[0].z };
	for ( int i = 1; i < count; ++i )
	{
		if ( points[i].x < minv[0] )
			minv[0] = points[i].x;
		if ( points[i].x > maxv[0] )
			maxv[0] = points[i].x;
		if ( points[i].y < minv[1] )
			minv[1] = points[i].y;
		if ( points[i].y > maxv[1] )
			maxv[1] = points[i].y;
		if ( points[i].z < minv[2] )
			minv[2] = points[i].z;
		if ( points[i].z > maxv[2] )
			maxv[2] = points[i].z;
	}
	float dx = maxv[0] - minv[0];
	float dy = maxv[1] - minv[1];
	float dz = maxv[2] - minv[2];
	return sqrtf( dx * dx + dy * dy + dz * dz );
}

void setBoundsFromPoints( SOP_Output* output, const Position* points, int count )
{
	if ( output == nullptr || points == nullptr || count <= 0 )
	{
		return;
	}
	BoundingBox bbox( points[0], points[0] );
	for ( int i = 1; i < count; ++i )
	{
		bbox.enlargeBounds( points[i] );
	}
	output->setBoundingBox( bbox );
}

void setBoundsFromLocalPoints( SOP_Output* output, const BodyTransform& t, const std::vector<float>& localPoints )
{
	if ( output == nullptr || localPoints.size() < 3 )
	{
		return;
	}

	b3Matrix3 r = rotationFromTransform( t );
	Position p0 = applyTransform( r, t, localPoints[0], localPoints[1], localPoints[2] );
	BoundingBox bbox( p0, p0 );
	for ( size_t i = 3; i + 2 < localPoints.size(); i += 3 )
	{
		Position p = applyTransform( r, t, localPoints[i], localPoints[i + 1], localPoints[i + 2] );
		bbox.enlargeBounds( p );
	}
	output->setBoundingBox( bbox );
}

// Append a box as 24 points (6 faces × 4), each face flat-shaded with a 0..1 UV
// so generated boxes/containers carry texture coordinates. Body-local box is
// centered at (ox,oy,oz) with the given half extents; transformed to world.
void appendBoxWithUV( SOP_Output* output, const b3Matrix3& r, const BodyTransform& t, float ox, float oy, float oz,
					  float hx, float hy, float hz, std::vector<Position>& worldPoints )
{
	const float nrm[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	const float verts[6][4][3] = {
		{ { hx, -hy, -hz }, { hx, hy, -hz }, { hx, hy, hz }, { hx, -hy, hz } },		 // +X
		{ { -hx, -hy, hz }, { -hx, hy, hz }, { -hx, hy, -hz }, { -hx, -hy, -hz } },	 // -X
		{ { -hx, hy, hz }, { hx, hy, hz }, { hx, hy, -hz }, { -hx, hy, -hz } },		 // +Y
		{ { -hx, -hy, -hz }, { hx, -hy, -hz }, { hx, -hy, hz }, { -hx, -hy, hz } },	 // -Y
		{ { -hx, -hy, hz }, { hx, -hy, hz }, { hx, hy, hz }, { -hx, hy, hz } },		 // +Z
		{ { hx, -hy, -hz }, { -hx, -hy, -hz }, { -hx, hy, -hz }, { hx, hy, -hz } },	 // -Z
	};
	const float uv[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };

	for ( int f = 0; f < 6; ++f )
	{
		int base = output->getNumPoints();
		for ( int k = 0; k < 4; ++k )
		{
			Position p = applyTransform( r, t, ox + verts[f][k][0], oy + verts[f][k][1], oz + verts[f][k][2] );
			output->addPoint( p );
			worldPoints.push_back( p );
			output->setNormal( rotateVector( r, nrm[f][0], nrm[f][1], nrm[f][2] ), base + k );
			TexCoord tc( uv[k][0], uv[k][1], 0.0f );
			output->setTexCoord( &tc, 1, base + k );
		}
		output->addTriangle( base + 0, base + 1, base + 2 );
		output->addTriangle( base + 0, base + 2, base + 3 );
	}
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
	// Required to kick-start cookEveryFrame on scene load: this node must
	// cook to (re)register its body — existence in the physics world cannot
	// depend on something watching the node.
	customInfo.cookOnStart = true;
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

Box3DBodySOP::Box3DBodySOP( const OP_NodeInfo* info ) : myOpId( info->opId ), myNodeInfo( info )
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
	// Cook unconditionally: this node's cook is what keeps its body
	// registered (and its heartbeat alive) in the physics world, whether or
	// not anything consumes the output. Bypassing the node stops its cooking
	// and the solver drops the body after a few silent steps.
	ginfo->cookEveryFrame = true;
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
	s.bullet = inputs->getParInt( BulletccdName ) != 0;
	s.wallThickness = (float)inputs->getParDouble( WallthicknessName );
	s.openTop = inputs->getParInt( OpentopName ) != 0;
	s.jointEnabled = inputs->getParInt( JointName ) != 0;
	inputs->getParDouble3( JointpivotName, x, y, z );
	s.jointPivotX = (float)x;
	s.jointPivotY = (float)y;
	s.jointPivotZ = (float)z;
	s.showJointPivot = inputs->getParInt( ShowjointpivotName ) != 0;
	s.density = (float)inputs->getParDouble( DensityName );
	s.friction = (float)inputs->getParDouble( FrictionName );
	s.restitution = (float)inputs->getParDouble( RestitutionName );
	return s;
}

bool Box3DBodySOP::applyJointAttributesFromInput( const OP_SOPInput* input, BodySettings& s ) const
{
	if ( input == nullptr )
	{
		return false;
	}

	const SOP_CustomAttribData* enabledAttr = input->getCustomAttribute( JointenabledAttrName );
	const SOP_CustomAttribData* pivotAttr = input->getCustomAttribute( JointpivotAttrName );
	if ( enabledAttr == nullptr && pivotAttr == nullptr )
	{
		return false;
	}

	if ( enabledAttr != nullptr && enabledAttr->numComponents >= 1 )
	{
		if ( enabledAttr->attribType == AttribType::Int && enabledAttr->intData != nullptr )
		{
			s.jointEnabled = enabledAttr->intData[0] != 0;
		}
		else if ( enabledAttr->attribType == AttribType::Float && enabledAttr->floatData != nullptr )
		{
			s.jointEnabled = enabledAttr->floatData[0] > 0.5f;
		}
	}

	if ( pivotAttr != nullptr && pivotAttr->numComponents >= 3 )
	{
		int pivotSpace = 0; // 0 = object/world input space, 1 = local body space
		const SOP_CustomAttribData* pivotSpaceAttr = input->getCustomAttribute( JointpivotspaceAttrName );
		if ( pivotSpaceAttr != nullptr && pivotSpaceAttr->numComponents >= 1 )
		{
			if ( pivotSpaceAttr->attribType == AttribType::Int && pivotSpaceAttr->intData != nullptr )
			{
				pivotSpace = pivotSpaceAttr->intData[0];
			}
			else if ( pivotSpaceAttr->attribType == AttribType::Float && pivotSpaceAttr->floatData != nullptr )
			{
				pivotSpace = pivotSpaceAttr->floatData[0] > 0.5f ? 1 : 0;
			}
		}

		float px = 0.0f;
		float py = 0.0f;
		float pz = 0.0f;
		if ( pivotAttr->attribType == AttribType::Float && pivotAttr->floatData != nullptr )
		{
			px = pivotAttr->floatData[0];
			py = pivotAttr->floatData[1];
			pz = pivotAttr->floatData[2];
			s.jointEnabled = true;
		}
		else if ( pivotAttr->attribType == AttribType::Int && pivotAttr->intData != nullptr )
		{
			px = (float)pivotAttr->intData[0];
			py = (float)pivotAttr->intData[1];
			pz = (float)pivotAttr->intData[2];
			s.jointEnabled = true;
		}

		const int pointCount = input->getNumPoints();
		const Position* points = input->getPointPositions();

		if ( pivotSpace == 1 )
		{
			// Legacy local pivot (offset from centroid): bring it back to the
			// input space so all paths hand the same thing downstream.
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
			px += (float)cx;
			py += (float)cy;
			pz += (float)cz;
		}

		// The Set Joint SOP also encodes the pivot relative to reference
		// points of the geometry (joint_ref + joint_ref_w). Rebuilding it from
		// the CURRENT point positions makes the pivot follow any SOP applied
		// between the Set Joint and this node (Transform, etc.); the raw
		// joint_pivot value is only the static fallback.
		const SOP_CustomAttribData* refAttr = input->getCustomAttribute( JointrefAttrName );
		const SOP_CustomAttribData* refWAttr = input->getCustomAttribute( JointrefwAttrName );
		if ( refAttr != nullptr && refAttr->numComponents >= 4 && refAttr->attribType == AttribType::Int &&
			 refAttr->intData != nullptr && refWAttr != nullptr && refWAttr->numComponents >= 3 &&
			 refWAttr->attribType == AttribType::Float && refWAttr->floatData != nullptr && points != nullptr )
		{
			const int32_t i0 = refAttr->intData[0];
			const int32_t i1 = refAttr->intData[1];
			const int32_t i2 = refAttr->intData[2];
			const int32_t i3 = refAttr->intData[3];
			const float a = refWAttr->floatData[0];
			const float b = refWAttr->floatData[1];
			const float c = refWAttr->floatData[2];

			if ( i0 >= 0 && i0 < pointCount )
			{
				const Position& p0 = points[i0];
				if ( i1 >= 0 && i1 < pointCount && i2 >= 0 && i2 < pointCount )
				{
					float e1[3] = { points[i1].x - p0.x, points[i1].y - p0.y, points[i1].z - p0.z };
					float e2[3] = { points[i2].x - p0.x, points[i2].y - p0.y, points[i2].z - p0.z };
					float e3[3];
					if ( i3 >= 0 && i3 < pointCount )
					{
						e3[0] = points[i3].x - p0.x;
						e3[1] = points[i3].y - p0.y;
						e3[2] = points[i3].z - p0.z;
					}
					else
					{
						// Planar encoding: third axis is the normalized plane normal.
						vecCross( e1, e2, e3 );
						normalizeVec( e3 );
					}
					px = p0.x + a * e1[0] + b * e2[0] + c * e3[0];
					py = p0.y + a * e1[1] + b * e2[1] + c * e3[1];
					pz = p0.z + a * e1[2] + b * e2[2] + c * e3[2];
				}
				else
				{
					px = p0.x + a;
					py = p0.y + b;
					pz = p0.z + c;
				}
			}
		}

		// Input/object-space pivot; execute() converts it to body-local once
		// the body origin (centroid) and spawn frame are known.
		s.jointPivotX = px;
		s.jointPivotY = py;
		s.jointPivotZ = pz;
	}

	return true;
}

void Box3DBodySOP::execute( SOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();

	const OP_SOPInput* input = inputs->getInputSOP( 0 );
	BodySettings settings = readSettings( inputs );
	bool hasInputJointAttrs = applyJointAttributesFromInput( input, settings );

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
		myInputFrameValid = false;
		myInputSourceQ[0] = 0.0f;
		myInputSourceQ[1] = 0.0f;
		myInputSourceQ[2] = 0.0f;
		myInputSourceQ[3] = 1.0f;
		myInputPointCount = 0;
		myHullLocalPoints.clear();

		// No simulation: show the input (or the primitive preview) untransformed
		myLastSettings = settings;
		// Geometry is drawn raw (centroid 0, identity pose), so the preview
		// pivot is the object-space value as-is.
		myJointPivotLocal[0] = settings.jointPivotX;
		myJointPivotLocal[1] = settings.jointPivotY;
		myJointPivotLocal[2] = settings.jointPivotZ;
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

	if ( myResetPending )
	{
		core->removeGroup( myOpId );
		myGroupRegistered = false;
		mySopId = 0;
		mySopCooks = -1;
		myInputFrameValid = false;
		myInputSourceQ[0] = 0.0f;
		myInputSourceQ[1] = 0.0f;
		myInputSourceQ[2] = 0.0f;
		myInputSourceQ[3] = 1.0f;
		myInputPointCount = 0;
		myHullLocalPoints.clear();
		myResetPending = false;
	}

	// Persistent (set every cook, not only on rebuild): mesh shape cannot be
	// dynamic — the engine only supports concave meshes on non-moving bodies.
	if ( settings.shape == 4 && settings.type == 0 && input != nullptr )
	{
		myWarning = "Mesh (Static) cannot be Dynamic (engine limit); the body is treated as Static. "
					"Use Input Hull for dynamic bodies.";
	}

	inputs->enablePar( PositionName, input == nullptr );
	inputs->enablePar( JointName, !hasInputJointAttrs );
	inputs->enablePar( JointpivotName, !hasInputJointAttrs && settings.jointEnabled );
	inputs->enablePar( ShowjointpivotName, !hasInputJointAttrs && settings.jointEnabled );
	// Container-only params.
	inputs->enablePar( WallthicknessName, settings.shape == 6 );
	inputs->enablePar( OpentopName, settings.shape == 6 );
	uint32_t sopId = input != nullptr ? input->opId : 0;
	int64_t sopCooks = input != nullptr ? input->totalCooks : -1;

	// Validate the cached hull reference against the LIVE cloud every cook,
	// outside any cook-count gating: an input that collapsed (scale through
	// zero/negative) and came back can otherwise leave the cache describing
	// geometry that no longer exists, and the node stays stuck until the
	// wire is re-connected. The diagonal is invariant under the rigid motion
	// the cache is meant to survive, so this never fires on animation.
	bool cacheInvalid = false;
	if ( input != nullptr && myInputFrameValid )
	{
		float diag = cloudDiagonal( input->getPointPositions(), input->getNumPoints() );
		float ref = myInputRefDiag;
		float tolerance = 0.01f * ( ref > diag ? ref : diag ) + 1e-6f;
		if ( fabsf( diag - ref ) > tolerance )
		{
			myInputFrameValid = false; // forces a reference rebuild below
			cacheInvalid = true;
		}
	}

	bool changed = !myGroupRegistered || cacheInvalid || settings != myLastSettings || sopId != mySopId ||
				   ( input != nullptr && sopCooks != mySopCooks );

	if ( changed )
	{
		SpawnBody def;
		def.type = 2 - settings.type; // menu dynamic/kinematic/static → core 2/1/0
		def.bullet = settings.bullet;
		def.density = settings.density;
		def.friction = settings.friction;
		def.restitution = settings.restitution;
		def.sizeX = settings.sizeX;
		def.sizeY = settings.sizeY;
		def.sizeZ = settings.sizeZ;
		def.wallThickness = settings.wallThickness;
		def.openTop = settings.openTop;

		if ( input != nullptr )
		{
			// Body origin = input centroid; derive rigid pose from the input
			// geometry frame so Transform SOP animation drives the body pose
			// without changing hull topology every frame.
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

			if ( def.shape == 3 && pointCount >= 3 )
			{
				bool rebuildReference = !myInputFrameValid || myInputPointCount != pointCount || sopId != mySopId;
				FrameBasis referenceFrame = {};
				FrameBasis currentFrame = {};

				if ( !rebuildReference )
				{
					if ( !buildFrameFromAnchors( points, pointCount, myInputAnchor, currentFrame ) )
					{
						rebuildReference = true;
					}
					else
					{
						float e0 = edgeLength( points[myInputAnchor[0]], points[myInputAnchor[1]] );
						float e1 = edgeLength( points[myInputAnchor[0]], points[myInputAnchor[2]] );
						float e2 = edgeLength( points[myInputAnchor[1]], points[myInputAnchor[2]] );
						float maxErr = fabsf( e0 - myInputRefEdgeLen[0] );
						if ( fabsf( e1 - myInputRefEdgeLen[1] ) > maxErr )
						{
							maxErr = fabsf( e1 - myInputRefEdgeLen[1] );
						}
						if ( fabsf( e2 - myInputRefEdgeLen[2] ) > maxErr )
						{
							maxErr = fabsf( e2 - myInputRefEdgeLen[2] );
						}
						if ( maxErr > 1e-4f )
						{
							rebuildReference = true;
						}
					}
				}

				if ( rebuildReference )
				{
					if ( chooseAnchorIndices( points, pointCount, myCentroid[0], myCentroid[1], myCentroid[2], myInputAnchor ) &&
						 buildFrameFromAnchors( points, pointCount, myInputAnchor, referenceFrame ) )
					{
						myInputFrameValid = true;
						myInputPointCount = pointCount;
						myInputRefDiag = cloudDiagonal( points, pointCount );
						myInputRefEdgeLen[0] = edgeLength( points[myInputAnchor[0]], points[myInputAnchor[1]] );
						myInputRefEdgeLen[1] = edgeLength( points[myInputAnchor[0]], points[myInputAnchor[2]] );
						myInputRefEdgeLen[2] = edgeLength( points[myInputAnchor[1]], points[myInputAnchor[2]] );

						myHullLocalPoints.clear();
						myHullLocalPoints.reserve( pointCount * 3 );
						for ( int i = 0; i < pointCount; ++i )
						{
							float rel[3] = { points[i].x - myCentroid[0], points[i].y - myCentroid[1], points[i].z - myCentroid[2] };
							myHullLocalPoints.push_back( vecDot( rel, referenceFrame.x ) );
							myHullLocalPoints.push_back( vecDot( rel, referenceFrame.y ) );
							myHullLocalPoints.push_back( vecDot( rel, referenceFrame.z ) );
						}

						currentFrame = referenceFrame;
					}
					else
					{
						myInputFrameValid = false;
						myInputPointCount = 0;
						myHullLocalPoints.clear();
					}
				}

				if ( myInputFrameValid && buildFrameFromAnchors( points, pointCount, myInputAnchor, currentFrame ) )
				{
					quatFromFrame( currentFrame, def.qx, def.qy, def.qz, def.qw );
					myInputSourceQ[0] = def.qx;
					myInputSourceQ[1] = def.qy;
					myInputSourceQ[2] = def.qz;
					myInputSourceQ[3] = def.qw;
				}
				else
				{
					myInputSourceQ[0] = 0.0f;
					myInputSourceQ[1] = 0.0f;
					myInputSourceQ[2] = 0.0f;
					myInputSourceQ[3] = 1.0f;
				}
			}
			else
			{
				// Primitive colliders with SOP input keep identity orientation by
				// default; using the anchor frame here can introduce arbitrary
				// initial rotations on symmetric meshes (for example a default Box SOP).
				myInputFrameValid = false;
				myInputSourceQ[0] = 0.0f;
				myInputSourceQ[1] = 0.0f;
				myInputSourceQ[2] = 0.0f;
				myInputSourceQ[3] = 1.0f;
				myInputPointCount = 0;
				myHullLocalPoints.clear();
			}


			if ( def.shape == 4 )
			{
				// Exact triangle mesh: concave geometry (terrain, tubes)
				// collides per-triangle. box3d only supports this on
				// non-dynamic bodies, so dynamic falls back to static here
				// (the persistent warning is set outside the rebuild block).
				if ( def.type == 2 )
				{
					def.type = 0;
				}

				def.hullPoints.reserve( pointCount * 3 );
				for ( int i = 0; i < pointCount; ++i )
				{
					def.hullPoints.push_back( points[i].x - myCentroid[0] );
					def.hullPoints.push_back( points[i].y - myCentroid[1] );
					def.hullPoints.push_back( points[i].z - myCentroid[2] );
				}

				// Oriented triangulation: box3d mesh contacts are one-sided,
				// so triangles must face the same way the geometry shades.
				extractOrientedTriangles( input, def.meshIndices );

				// Keep the box fallback sizes in sync with the geometry bounds
				// in case the mesh cannot be built.
				if ( def.hullPoints.size() >= 3 )
				{
					float minX = def.hullPoints[0], maxX = def.hullPoints[0];
					float minY = def.hullPoints[1], maxY = def.hullPoints[1];
					float minZ = def.hullPoints[2], maxZ = def.hullPoints[2];
					for ( size_t i = 3; i + 2 < def.hullPoints.size(); i += 3 )
					{
						if ( def.hullPoints[i] < minX )
							minX = def.hullPoints[i];
						if ( def.hullPoints[i] > maxX )
							maxX = def.hullPoints[i];
						if ( def.hullPoints[i + 1] < minY )
							minY = def.hullPoints[i + 1];
						if ( def.hullPoints[i + 1] > maxY )
							maxY = def.hullPoints[i + 1];
						if ( def.hullPoints[i + 2] < minZ )
							minZ = def.hullPoints[i + 2];
						if ( def.hullPoints[i + 2] > maxZ )
							maxZ = def.hullPoints[i + 2];
					}

					constexpr float kMinExtent = 0.001f;
					def.sizeX = ( maxX - minX ) > kMinExtent ? ( maxX - minX ) : kMinExtent;
					def.sizeY = ( maxY - minY ) > kMinExtent ? ( maxY - minY ) : kMinExtent;
					def.sizeZ = ( maxZ - minZ ) > kMinExtent ? ( maxZ - minZ ) : kMinExtent;
				}
			}
			else if ( def.shape == 5 )
			{
				// Compound: one convex hull per connectivity island of the
				// input (points shared by primitives weld pieces together).
				// Model concave objects in pieces — tube segments of a torus,
				// legs of a chair — and the body can stay dynamic.
				std::vector<int32_t> parent( pointCount );
				for ( int i = 0; i < pointCount; ++i )
				{
					parent[i] = i;
				}
				std::vector<bool> used( pointCount, false );

				int primCount = input->getNumPrimitives();
				for ( int i = 0; i < primCount; ++i )
				{
					const SOP_PrimitiveInfo& prim = input->getPrimitive( i );
					if ( prim.type != PrimitiveType::Polygon || prim.numVertices < 3 )
					{
						continue;
					}
					for ( int v = 0; v < prim.numVertices; ++v )
					{
						int32_t idx = prim.pointIndices[v];
						if ( idx < 0 || idx >= pointCount )
						{
							continue;
						}
						used[idx] = true;
						unionRoots( parent, prim.pointIndices[0], idx );
					}
				}

				// Bucket points per island root, preserving discovery order.
				std::vector<int32_t> islandOfRoot( pointCount, -1 );
				std::vector<std::vector<int32_t>> islands;
				for ( int i = 0; i < pointCount; ++i )
				{
					if ( !used[i] )
					{
						continue;
					}
					int32_t root = findRoot( parent, i );
					if ( islandOfRoot[root] < 0 )
					{
						islandOfRoot[root] = (int32_t)islands.size();
						islands.emplace_back();
					}
					islands[islandOfRoot[root]].push_back( i );
				}

				constexpr int kMaxPieces = 64;
				if ( (int)islands.size() > kMaxPieces )
				{
					myWarning = "Compound: more than 64 connectivity islands; extra pieces merged into the last hull.";
					for ( size_t k = kMaxPieces; k < islands.size(); ++k )
					{
						islands[kMaxPieces - 1].insert( islands[kMaxPieces - 1].end(), islands[k].begin(),
														islands[k].end() );
					}
					islands.resize( kMaxPieces );
				}

				def.hullPoints.reserve( pointCount * 3 );
				for ( const std::vector<int32_t>& island : islands )
				{
					if ( (int)island.size() < 4 )
					{
						continue; // degenerate piece, no volume
					}
					for ( int32_t idx : island )
					{
						def.hullPoints.push_back( points[idx].x - myCentroid[0] );
						def.hullPoints.push_back( points[idx].y - myCentroid[1] );
						def.hullPoints.push_back( points[idx].z - myCentroid[2] );
					}
					def.hullPieceCounts.push_back( (int32_t)island.size() );
				}

				// Fallback box sizes track the geometry bounds.
				if ( def.hullPoints.size() >= 3 )
				{
					float minX = def.hullPoints[0], maxX = def.hullPoints[0];
					float minY = def.hullPoints[1], maxY = def.hullPoints[1];
					float minZ = def.hullPoints[2], maxZ = def.hullPoints[2];
					for ( size_t i = 3; i + 2 < def.hullPoints.size(); i += 3 )
					{
						if ( def.hullPoints[i] < minX )
							minX = def.hullPoints[i];
						if ( def.hullPoints[i] > maxX )
							maxX = def.hullPoints[i];
						if ( def.hullPoints[i + 1] < minY )
							minY = def.hullPoints[i + 1];
						if ( def.hullPoints[i + 1] > maxY )
							maxY = def.hullPoints[i + 1];
						if ( def.hullPoints[i + 2] < minZ )
							minZ = def.hullPoints[i + 2];
						if ( def.hullPoints[i + 2] > maxZ )
							maxZ = def.hullPoints[i + 2];
					}

					constexpr float kMinExtent = 0.001f;
					def.sizeX = ( maxX - minX ) > kMinExtent ? ( maxX - minX ) : kMinExtent;
					def.sizeY = ( maxY - minY ) > kMinExtent ? ( maxY - minY ) : kMinExtent;
					def.sizeZ = ( maxZ - minZ ) > kMinExtent ? ( maxZ - minZ ) : kMinExtent;
				}
			}
			else if ( def.shape == 3 )
			{
				if ( (int)myHullLocalPoints.size() == pointCount * 3 )
				{
					def.hullPoints = myHullLocalPoints;
				}
				else
				{
					def.hullPoints.reserve( pointCount * 3 );
					for ( int i = 0; i < pointCount; ++i )
					{
						def.hullPoints.push_back( points[i].x - myCentroid[0] );
						def.hullPoints.push_back( points[i].y - myCentroid[1] );
						def.hullPoints.push_back( points[i].z - myCentroid[2] );
					}
				}

				// If hull creation degenerates, core falls back to a box using sizeX/Y/Z.
				// Keep these in sync with the local hull bounds so collision matches
				// the visible geometry orientation/extent.
				if ( def.hullPoints.size() >= 3 )
				{
					float minX = def.hullPoints[0], maxX = def.hullPoints[0];
					float minY = def.hullPoints[1], maxY = def.hullPoints[1];
					float minZ = def.hullPoints[2], maxZ = def.hullPoints[2];
					for ( size_t i = 3; i + 2 < def.hullPoints.size(); i += 3 )
					{
						if ( def.hullPoints[i] < minX )
							minX = def.hullPoints[i];
						if ( def.hullPoints[i] > maxX )
							maxX = def.hullPoints[i];
						if ( def.hullPoints[i + 1] < minY )
							minY = def.hullPoints[i + 1];
						if ( def.hullPoints[i + 1] > maxY )
							maxY = def.hullPoints[i + 1];
						if ( def.hullPoints[i + 2] < minZ )
							minZ = def.hullPoints[i + 2];
						if ( def.hullPoints[i + 2] > maxZ )
							maxZ = def.hullPoints[i + 2];
					}

					constexpr float kMinExtent = 0.001f;
					def.sizeX = ( maxX - minX ) > kMinExtent ? ( maxX - minX ) : kMinExtent;
					def.sizeY = ( maxY - minY ) > kMinExtent ? ( maxY - minY ) : kMinExtent;
					def.sizeZ = ( maxZ - minZ ) > kMinExtent ? ( maxZ - minZ ) : kMinExtent;
				}
			}
		}
		else
		{
			// No input: primitive at the Position parameter ("Input Hull"
			// falls back to a box)
			myInputFrameValid = false;
			myInputSourceQ[0] = 0.0f;
			myInputSourceQ[1] = 0.0f;
			myInputSourceQ[2] = 0.0f;
			myInputSourceQ[3] = 1.0f;
			myInputPointCount = 0;
			myHullLocalPoints.clear();
			myCentroid[0] = myCentroid[1] = myCentroid[2] = 0.0f;
			def.px = settings.posX;
			def.py = settings.posY;
			def.pz = settings.posZ;
			int coreShape = menuShapeToCoreShape( settings.shape );
			// Hull/mesh/compound need input geometry; all preview as a box.
			def.shape = ( coreShape == 3 || coreShape == 4 || coreShape == 5 ) ? 0 : coreShape;
		}

		def.jointEnabled = settings.jointEnabled;

		// Convert the pivot to body-local exactly once, now that the body
		// origin and spawn orientation are final. Attribute pivots come in
		// input/object space (absolute); the manual parameter is an offset
		// from the body origin in world axes. Either way the core stores the
		// pivot in the body frame and re-rotates it with the live pose, so it
		// must be expressed in the same frame as the hull points (def.q).
		{
			float offX = settings.jointPivotX;
			float offY = settings.jointPivotY;
			float offZ = settings.jointPivotZ;
			if ( input != nullptr && hasInputJointAttrs )
			{
				offX -= myCentroid[0];
				offY -= myCentroid[1];
				offZ -= myCentroid[2];
			}

			b3Matrix3 bodyR = rotationFromQuat( def.qx, def.qy, def.qz, def.qw );
			float lx = bodyR.cx.x * offX + bodyR.cx.y * offY + bodyR.cx.z * offZ;
			float ly = bodyR.cy.x * offX + bodyR.cy.y * offY + bodyR.cy.z * offZ;
			float lz = bodyR.cz.x * offX + bodyR.cz.y * offY + bodyR.cz.z * offZ;

			// Under rigid upstream animation the local pivot is invariant up to
			// float noise; snap to the previous value so the core's exact spec
			// compare does not flag the joints dirty (full re-sync) every frame.
			constexpr float kPivotSnap = 1e-4f;
			if ( fabsf( lx - myJointPivotLocal[0] ) < kPivotSnap && fabsf( ly - myJointPivotLocal[1] ) < kPivotSnap &&
				 fabsf( lz - myJointPivotLocal[2] ) < kPivotSnap )
			{
				lx = myJointPivotLocal[0];
				ly = myJointPivotLocal[1];
				lz = myJointPivotLocal[2];
			}

			myJointPivotLocal[0] = lx;
			myJointPivotLocal[1] = ly;
			myJointPivotLocal[2] = lz;
			def.jointPivotX = lx;
			def.jointPivotY = ly;
			def.jointPivotZ = lz;
		}

		core->setGroup( myOpId, { def } );
		myGroupRegistered = true;
		myLastSettings = settings;
		mySopId = sopId;
		mySopCooks = sopCooks;
	}

	// Keep the node path registered (it changes on rename/move) so the
	// solver's Joints DAT can reference this group.
	core->setGroupPath( myOpId, myNodeInfo->opPath );

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

	// Debug overlay: the REAL collider box3d is using for this body (hull
	// after the vertex budget, mesh after welding), at the live pose, colored
	// by state (dynamic awake green / asleep blue, kinematic orange, static
	// gray). Appended to the output like the joint pivot marker.
	if ( inputs->getParInt( ShowcollisionName ) != 0 )
	{
		std::vector<float> segments;
		std::vector<float> colors;
		core->getGroupWireframe( myOpId, segments, colors );
		int segCount = (int)( segments.size() / 6 );
		for ( int i = 0; i < segCount; ++i )
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
		}
	}
}

void Box3DBodySOP::outputTransformedInput( SOP_Output* output, const OP_SOPInput* input, const BodyTransform& t )
{
	b3Matrix3 r = rotationFromTransform( t );

	int pointCount = input->getNumPoints();
	const Position* points = input->getPointPositions();
	bool useCachedLocalHull = ( myLastSettings.shape == 0 ) && myInputFrameValid &&
									( (int)myHullLocalPoints.size() == pointCount * 3 );
	const SOP_NormalInfo* normalInfo = input->hasNormals() ? input->getNormals() : nullptr;
	bool hasPointNormals = normalInfo != nullptr && normalInfo->normals != nullptr &&
					 normalInfo->attribSet == AttribSet::Point && normalInfo->numNormals >= pointCount;
	bool hasVertexNormals = normalInfo != nullptr && normalInfo->normals != nullptr &&
					  normalInfo->attribSet == AttribSet::Vertex && normalInfo->numNormals >= input->getNumVertices();
	bool hasPrimitiveNormals = normalInfo != nullptr && normalInfo->normals != nullptr &&
						 normalInfo->attribSet == AttribSet::Primitive && normalInfo->numNormals >= input->getNumPrimitives();

	// Texture coordinates pass through untouched (only positions/normals
	// rotate with the body).
	const SOP_TextureInfo* texInfo = input->getTextures();
	int texLayers = texInfo != nullptr ? texInfo->numTextureLayers : 0;
	bool hasPointTex = texInfo != nullptr && texInfo->textures != nullptr && texLayers > 0 &&
					   texInfo->attribSet == AttribSet::Point && texInfo->numTextures >= pointCount;
	bool hasVertexTex = texInfo != nullptr && texInfo->textures != nullptr && texLayers > 0 &&
						texInfo->attribSet == AttribSet::Vertex && texInfo->numTextures >= input->getNumVertices();

	b3Matrix3 normalRot = r;
	if ( useCachedLocalHull )
	{
		b3Matrix3 sourceR = rotationFromQuat( myInputSourceQ[0], myInputSourceQ[1], myInputSourceQ[2], myInputSourceQ[3] );
		b3Matrix3 sourceInv = transposeMatrix( sourceR );
		normalRot = multiplyMatrix( r, sourceInv );
	}

	auto getLocalPoint = [&]( int srcPointIndex, float& lx, float& ly, float& lz ) {
		if ( useCachedLocalHull )
		{
			lx = myHullLocalPoints[srcPointIndex * 3 + 0];
			ly = myHullLocalPoints[srcPointIndex * 3 + 1];
			lz = myHullLocalPoints[srcPointIndex * 3 + 2];
		}
		else
		{
			lx = points[srcPointIndex].x - myCentroid[0];
			ly = points[srcPointIndex].y - myCentroid[1];
			lz = points[srcPointIndex].z - myCentroid[2];
		}
	};

	// Vertex/primitive normals (and vertex texture coordinates) cannot be
	// represented on shared points without smoothing/seam artifacts. Expand
	// points per primitive vertex to preserve them.
	if ( hasVertexNormals || hasPrimitiveNormals || hasVertexTex )
	{
		std::vector<Position> worldPoints;
		worldPoints.reserve( input->getNumVertices() );

		int primCount = input->getNumPrimitives();
		for ( int i = 0; i < primCount; ++i )
		{
			const SOP_PrimitiveInfo& prim = input->getPrimitive( i );
			if ( prim.type != PrimitiveType::Polygon || prim.numVertices < 3 )
			{
				continue;
			}

			std::vector<int32_t> remapped;
			remapped.reserve( prim.numVertices );

			for ( int v = 0; v < prim.numVertices; ++v )
			{
				int srcPointIndex = prim.pointIndices[v];
				float lx, ly, lz;
				getLocalPoint( srcPointIndex, lx, ly, lz );

				Position p = applyTransform( r, t, lx, ly, lz );
				int32_t outPointIndex = output->addPoint( p );
				worldPoints.push_back( p );

				if ( hasVertexNormals )
				{
					int normalIndex = prim.pointIndicesOffset + v;
					const Vector& inN = normalInfo->normals[normalIndex];
					output->setNormal( rotateVector( normalRot, inN.x, inN.y, inN.z ), outPointIndex );
				}
				else if ( hasPrimitiveNormals )
				{
					const Vector& inN = normalInfo->normals[i];
					output->setNormal( rotateVector( normalRot, inN.x, inN.y, inN.z ), outPointIndex );
				}
				else if ( hasPointNormals )
				{
					const Vector& inN = normalInfo->normals[srcPointIndex];
					output->setNormal( rotateVector( normalRot, inN.x, inN.y, inN.z ), outPointIndex );
				}

				if ( hasVertexTex )
				{
					output->setTexCoord( &texInfo->textures[( prim.pointIndicesOffset + v ) * texLayers], texLayers,
										 outPointIndex );
				}
				else if ( hasPointTex )
				{
					output->setTexCoord( &texInfo->textures[srcPointIndex * texLayers], texLayers, outPointIndex );
				}
				remapped.push_back( outPointIndex );
			}

			for ( int v = 1; v < prim.numVertices - 1; ++v )
			{
				output->addTriangle( remapped[0], remapped[v], remapped[v + 1] );
			}
		}

		if ( !worldPoints.empty() )
		{
			addJointPreviewMarker( output, myLastSettings.jointEnabled, myLastSettings.showJointPivot, myJointPivotLocal[0],
								   myJointPivotLocal[1], myJointPivotLocal[2], myLastSettings.sizeX,
								   myLastSettings.sizeY, myLastSettings.sizeZ, t, worldPoints );
			setBoundsFromPoints( output, worldPoints.data(), (int)worldPoints.size() );
		}

		return;
	}

	for ( int i = 0; i < pointCount; ++i )
	{
		float lx, ly, lz;
		getLocalPoint( i, lx, ly, lz );

		Position p = applyTransform( r, t, lx, ly, lz );
		output->addPoint( p );
	}

	if ( pointCount > 0 )
	{
		std::vector<Position> worldPoints;
		worldPoints.reserve( pointCount );
		for ( int i = 0; i < pointCount; ++i )
		{
			float lx, ly, lz;
			getLocalPoint( i, lx, ly, lz );
			worldPoints.push_back( applyTransform( r, t, lx, ly, lz ) );
		}
		addJointPreviewMarker( output, myLastSettings.jointEnabled, myLastSettings.showJointPivot, myJointPivotLocal[0],
							   myJointPivotLocal[1], myJointPivotLocal[2], myLastSettings.sizeX,
							   myLastSettings.sizeY, myLastSettings.sizeZ, t, worldPoints );
		setBoundsFromPoints( output, worldPoints.data(), pointCount );
	}

	if ( hasPointNormals )
	{
		for ( int i = 0; i < pointCount; ++i )
		{
			const Vector& n = normalInfo->normals[i];
			output->setNormal( rotateVector( normalRot, n.x, n.y, n.z ), i );
		}
	}

	if ( hasPointTex && pointCount > 0 )
	{
		output->setTexCoords( texInfo->textures, pointCount, texLayers, 0 );
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
	if ( coreShape == 3 || coreShape == 4 || coreShape == 5 )
	{
		coreShape = 0; // no input: hull/mesh/compound preview as a box
	}

	if ( coreShape == 6 )
	{
		// Container: the same six (or five) wall slabs box3d collides with, so
		// you see the crate and its inward-facing cavity. Open Top drops the lid.
		float hx = 0.5f * s.sizeX, hy = 0.5f * s.sizeY, hz = 0.5f * s.sizeZ;
		float tt = s.wallThickness > 0.001f ? s.wallThickness : 0.001f;
		float ht = 0.5f * tt;
		if ( ht > hx * 0.98f )
			ht = hx * 0.98f;
		if ( ht > hy * 0.98f )
			ht = hy * 0.98f;
		if ( ht > hz * 0.98f )
			ht = hz * 0.98f;

		std::vector<Position> worldPoints;
		appendBoxWithUV( output, r, t, hx - ht, 0.0f, 0.0f, ht, hy, hz, worldPoints );		 // +X
		appendBoxWithUV( output, r, t, -( hx - ht ), 0.0f, 0.0f, ht, hy, hz, worldPoints );	 // -X
		appendBoxWithUV( output, r, t, 0.0f, 0.0f, hz - ht, hx, hy, ht, worldPoints );		 // +Z
		appendBoxWithUV( output, r, t, 0.0f, 0.0f, -( hz - ht ), hx, hy, ht, worldPoints );	 // -Z
		appendBoxWithUV( output, r, t, 0.0f, -( hy - ht ), 0.0f, hx, ht, hz, worldPoints );	 // -Y floor
		if ( !s.openTop )
			appendBoxWithUV( output, r, t, 0.0f, hy - ht, 0.0f, hx, ht, hz, worldPoints ); // +Y lid

		if ( !worldPoints.empty() )
		{
			addJointPreviewMarker( output, s.jointEnabled, s.showJointPivot, s.jointPivotX, s.jointPivotY,
								   s.jointPivotZ, s.sizeX, s.sizeY, s.sizeZ, t, worldPoints );
			setBoundsFromPoints( output, worldPoints.data(), (int)worldPoints.size() );
		}
		return;
	}

	if ( coreShape == 0 )
	{
		// Box: 24 points (per-face) so it carries flat normals and 0..1 UVs.
		float hx = 0.5f * s.sizeX, hy = 0.5f * s.sizeY, hz = 0.5f * s.sizeZ;
		std::vector<Position> worldPoints;
		worldPoints.reserve( 24 );
		appendBoxWithUV( output, r, t, 0.0f, 0.0f, 0.0f, hx, hy, hz, worldPoints );
		addJointPreviewMarker( output, s.jointEnabled, s.showJointPivot, s.jointPivotX, s.jointPivotY, s.jointPivotZ,
							   s.sizeX, s.sizeY, s.sizeZ, t, worldPoints );
		setBoundsFromPoints( output, worldPoints.data(), (int)worldPoints.size() );
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
			TexCoord tc( (float)seg / (float)kSegments, 1.0f - (float)ring / (float)kRings, 0.0f );
			output->setTexCoord( &tc, 1, pointIndex );
			++pointIndex;
		}
	}

	// Coarse but stable bounds for sphere/capsule preview.
	float halfY = coreShape == 2 ? ( 0.5f * s.sizeY > radius ? 0.5f * s.sizeY : radius ) : radius;
	std::vector<float> localBoundsPts = {
		-radius, -halfY, -radius, radius, -halfY, -radius, radius, halfY, -radius, -radius, halfY, -radius,
		-radius, -halfY, radius,  radius, -halfY, radius,  radius, halfY, radius,  -radius, halfY, radius
	};
	setBoundsFromLocalPoints( output, t, localBoundsPts );

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
	return 16;
}

void Box3DBodySOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	static const char* names[16] = { "tx", "ty", "tz", "rx", "ry", "rz", "vx", "vy",
									 "vz", "wx", "wy", "wz", "awake", "touching", "impulse", "hitspeed" };
	chan->name->setString( names[index] );

	if ( index < 3 )
	{
		const float pos[3] = { myTransform.px, myTransform.py, myTransform.pz };
		chan->value = pos[index];
		return;
	}

	if ( index < 6 )
	{
		float rx, ry, rz;
		quatToEulerXYZDegrees( myTransform.qx, myTransform.qy, myTransform.qz, myTransform.qw, &rx, &ry, &rz );
		const float rot[3] = { rx, ry, rz };
		chan->value = rot[index - 3];
		return;
	}

	SolverCore* core = mySolverOpId != 0 ? Registry::find( mySolverOpId ) : nullptr;

	// Contact state: touching count, summed normal impulse, and last hit
	// speed (nonzero for one frame per impact — trigger material).
	if ( index >= 13 )
	{
		tdb3::BodyContactState contact = {};
		if ( core != nullptr )
		{
			core->getGroupContactStates( myOpId, &contact, 1 );
		}
		const float vals[3] = { contact.touching, contact.impulse, contact.hitSpeed };
		chan->value = vals[index - 13];
		return;
	}

	// Velocities and awake state, straight from the live body (zeros when the
	// solver is missing). Angular velocity in deg/s.
	constexpr float kRadToDeg = 57.295779513082320877f;
	tdb3::BodyState state = {};
	if ( core != nullptr )
	{
		core->getGroupStates( myOpId, &state, 1 );
	}

	const float rest[7] = { state.vx, state.vy, state.vz, state.wx * kRadToDeg, state.wy * kRadToDeg,
							state.wz * kRadToDeg, state.awake };
	chan->value = rest[index - 6];
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
		// Auto-bind: a sibling solver with TD's default name resolves on
		// creation (paths are relative to this node).
		p.defaultValue = "Box3dsolver1";
		manager->appendCHOP( p );
	}

	{
		OP_NumericParameter p;
		p.name = ResetName;
		p.label = "Reset";
		p.page = "Body";
		manager->appendPulse( p );
	}

	{
		OP_StringParameter p;
		p.name = ShapeName;
		p.label = "Shape";
		p.page = "Body";
		p.defaultValue = "inputhull";
		const char* names[] = { "inputhull", "box", "sphere", "capsule", "mesh", "compound", "container" };
		const char* labels[] = { "Input Hull", "Box",	"Sphere",	 "Capsule",
								 "Mesh (Static)", "Compound (Hulls)", "Box (Container, Inward)" };
		manager->appendMenu( p, 7, names, labels );
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

	// Joint pivot lives on its own page: it is only relevant when a Joint
	// CHOP references this body, and it is overridden (and grayed out) by
	// Set Joint SOP attributes coming in through the input.
	{
		OP_NumericParameter p;
		p.name = JointName;
		p.label = "Joint";
		p.page = "Joint";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = JointpivotName;
		p.label = "Joint Pivot";
		p.page = "Joint";
		for ( int i = 0; i < 3; ++i )
		{
			p.minSliders[i] = -10.0;
			p.maxSliders[i] = 10.0;
		}
		manager->appendXYZ( p );
	}

	{
		OP_NumericParameter p;
		p.name = ShowjointpivotName;
		p.label = "Show Joint Pivot";
		p.page = "Joint";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
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
		p.name = BulletccdName;
		p.label = "CCD (Bullet)";
		p.page = "Body";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		// Container shape only: thickness of the six inward-facing wall slabs.
		OP_NumericParameter p;
		p.name = WallthicknessName;
		p.label = "Wall Thickness";
		p.page = "Body";
		p.defaultValues[0] = 0.1;
		p.minValues[0] = 0.001;
		p.minSliders[0] = 0.01;
		p.maxSliders[0] = 1.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		// Container shape only: leave the top (+Y) face open to drop things in.
		OP_NumericParameter p;
		p.name = OpentopName;
		p.label = "Open Top";
		p.page = "Body";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = ShowcollisionName;
		p.label = "Show Collision Shape";
		p.page = "Body";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
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

}

void Box3DBodySOP::pulsePressed( const char* name, void* )
{
	if ( std::strcmp( name, ResetName ) == 0 )
	{
		myResetPending = true;
	}
}

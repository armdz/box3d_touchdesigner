#include "Box3DSetJointSOP.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace
{

constexpr char EnableName[] = "Enable";
constexpr char AnchorModeName[] = "Anchormode";
constexpr char AnchorName[] = "Anchor";
constexpr char ShowAnchorName[] = "Showanchor";
constexpr char AnchorcolorName[] = "Anchorcolor";

constexpr char kAttrJointEnabled[] = "joint_enabled";
constexpr char kAttrJointPivot[] = "joint_pivot";
constexpr char kAttrJointPivotSpace[] = "joint_pivot_space";
constexpr char kAttrJointRef[] = "joint_ref";
constexpr char kAttrJointRefW[] = "joint_ref_w";

void computeBounds( const Position* points, int pointCount, float minv[3], float maxv[3] )
{
	if ( points == nullptr || pointCount <= 0 )
	{
		minv[0] = minv[1] = minv[2] = 0.0f;
		maxv[0] = maxv[1] = maxv[2] = 0.0f;
		return;
	}

	minv[0] = maxv[0] = points[0].x;
	minv[1] = maxv[1] = points[0].y;
	minv[2] = maxv[2] = points[0].z;

	for ( int i = 1; i < pointCount; ++i )
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
}

void setAnchorFromMode( int mode, const float minv[3], const float maxv[3], float out[3] )
{
	const float cx = 0.5f * ( minv[0] + maxv[0] );
	const float cy = 0.5f * ( minv[1] + maxv[1] );
	const float cz = 0.5f * ( minv[2] + maxv[2] );

	out[0] = cx;
	out[1] = cy;
	out[2] = cz;

	switch ( mode )
	{
		case 1:
			out[0] = maxv[0];
			break; // tip x+
		case 2:
			out[0] = minv[0];
			break; // tip x-
		case 3:
			out[1] = maxv[1];
			break; // tip y+
		case 4:
			out[1] = minv[1];
			break; // tip y-
		case 5:
			out[2] = maxv[2];
			break; // tip z+
		case 6:
			out[2] = minv[2];
			break; // tip z-
		default:
			break;
	}
}

void setBoundsFromMinMax( SOP_Output* output, const float minv[3], const float maxv[3] )
{
	BoundingBox bbox( Position( minv[0], minv[1], minv[2] ), Position( maxv[0], maxv[1], maxv[2] ) );
	output->setBoundingBox( bbox );
}

inline void refCross( const float a[3], const float b[3], float out[3] )
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

inline float refDot( const float a[3], const float b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void refEdge( const Position& to, const Position& from, float out[3] )
{
	out[0] = to.x - from.x;
	out[1] = to.y - from.y;
	out[2] = to.z - from.z;
}

// Picks up to 4 reference points spanning the geometry: p0 far from the
// centroid, p1 far from p0, p2 maximizing triangle area, p3 maximizing
// tetrahedron volume. Returns how many are usable (4, 3, 1 or 0); unused
// slots are -1.
int chooseRefPoints( const Position* points, int pointCount, const float centroid[3], int idx[4] )
{
	idx[0] = idx[1] = idx[2] = idx[3] = -1;
	if ( points == nullptr || pointCount <= 0 )
	{
		return 0;
	}

	float best = -1.0f;
	for ( int i = 0; i < pointCount; ++i )
	{
		float d[3] = { points[i].x - centroid[0], points[i].y - centroid[1], points[i].z - centroid[2] };
		float d2 = refDot( d, d );
		if ( d2 > best )
		{
			best = d2;
			idx[0] = i;
		}
	}

	best = 1e-10f;
	for ( int i = 0; i < pointCount; ++i )
	{
		float d[3];
		refEdge( points[i], points[idx[0]], d );
		float d2 = refDot( d, d );
		if ( d2 > best )
		{
			best = d2;
			idx[1] = i;
		}
	}
	if ( idx[1] < 0 )
	{
		return 1;
	}

	float e1[3];
	refEdge( points[idx[1]], points[idx[0]], e1 );

	best = 1e-12f;
	for ( int i = 0; i < pointCount; ++i )
	{
		float d[3], c[3];
		refEdge( points[i], points[idx[0]], d );
		refCross( e1, d, c );
		float areaSq = refDot( c, c );
		if ( areaSq > best )
		{
			best = areaSq;
			idx[2] = i;
		}
	}
	if ( idx[2] < 0 )
	{
		idx[1] = -1;
		return 1;
	}

	float e2[3], n[3];
	refEdge( points[idx[2]], points[idx[0]], e2 );
	refCross( e1, e2, n );

	best = 1e-12f;
	for ( int i = 0; i < pointCount; ++i )
	{
		float d[3];
		refEdge( points[i], points[idx[0]], d );
		float vol = fabsf( refDot( n, d ) );
		if ( vol > best )
		{
			best = vol;
			idx[3] = i;
		}
	}

	return idx[3] >= 0 ? 4 : 3;
}

// Solves d = a*e1 + b*e2 + c*e3 (Cramer). False when the basis is degenerate.
bool solveRefCoords( const float e1[3], const float e2[3], const float e3[3], const float d[3], float out[3] )
{
	float c23[3], c31[3], c12[3];
	refCross( e2, e3, c23 );
	refCross( e3, e1, c31 );
	refCross( e1, e2, c12 );
	float det = refDot( e1, c23 );
	if ( fabsf( det ) < 1e-12f )
	{
		return false;
	}
	float inv = 1.0f / det;
	out[0] = refDot( d, c23 ) * inv;
	out[1] = refDot( d, c31 ) * inv;
	out[2] = refDot( d, c12 ) * inv;
	return true;
}

void computeCentroid( const Position* points, int pointCount, float out[3] )
{
	out[0] = out[1] = out[2] = 0.0f;
	if ( points == nullptr || pointCount <= 0 )
	{
		return;
	}

	for ( int i = 0; i < pointCount; ++i )
	{
		out[0] += points[i].x;
		out[1] += points[i].y;
		out[2] += points[i].z;
	}
	float inv = 1.0f / (float)pointCount;
	out[0] *= inv;
	out[1] *= inv;
	out[2] *= inv;
}

} // namespace

extern "C"
{

DLLEXPORT
void FillSOPPluginInfo( SOP_PluginInfo* info )
{
	info->apiVersion = SOPCPlusPlusAPIVersion;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString( "Box3dsetjoint" );
	customInfo.opLabel->setString( "Box3D Set Joint" );
	customInfo.opIcon->setString( "SJT" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 1;
	customInfo.maxInputs = 1;
}

DLLEXPORT
SOP_CPlusPlusBase* CreateSOPInstance( const OP_NodeInfo* info )
{
	return new Box3DSetJointSOP( info );
}

DLLEXPORT
void DestroySOPInstance( SOP_CPlusPlusBase* instance )
{
	delete (Box3DSetJointSOP*)instance;
}

} // extern "C"

Box3DSetJointSOP::Box3DSetJointSOP( const OP_NodeInfo* )
{
}

Box3DSetJointSOP::~Box3DSetJointSOP()
{
}

void Box3DSetJointSOP::getGeneralInfo( SOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->directToGPU = false;
}

void Box3DSetJointSOP::execute( SOP_Output* output, const OP_Inputs* inputs, void* )
{
	const OP_SOPInput* input = inputs->getInputSOP( 0 );
	if ( input == nullptr )
	{
		myEnabled = false;
		myPivot[0] = myPivot[1] = myPivot[2] = 0.0f;
		return;
	}

	myEnabled = inputs->getParInt( EnableName ) != 0;
	int mode = inputs->getParInt( AnchorModeName );
	bool showAnchor = inputs->getParInt( ShowAnchorName ) != 0;
	double cr, cg, cb;
	inputs->getParDouble3( AnchorcolorName, cr, cg, cb );

	double ax, ay, az;
	inputs->getParDouble3( AnchorName, ax, ay, az );

	const int pointCount = input->getNumPoints();
	const Position* points = input->getPointPositions();

	float minv[3], maxv[3];
	computeBounds( points, pointCount, minv, maxv );

	float worldPivot[3] = { 0.0f, 0.0f, 0.0f };
	if ( mode == 7 )
	{
		worldPivot[0] = (float)ax;
		worldPivot[1] = (float)ay;
		worldPivot[2] = (float)az;
	}
	else
	{
		setAnchorFromMode( mode, minv, maxv, worldPivot );
	}

	float centroid[3] = { 0.0f, 0.0f, 0.0f };
	computeCentroid( points, pointCount, centroid );
	myPivot[0] = worldPivot[0] - centroid[0];
	myPivot[1] = worldPivot[1] - centroid[1];
	myPivot[2] = worldPivot[2] - centroid[2];

	inputs->enablePar( AnchorName, mode == 7 );

	for ( int i = 0; i < pointCount; ++i )
	{
		output->addPoint( points[i] );
	}

	const SOP_NormalInfo* normalInfo = input->hasNormals() ? input->getNormals() : nullptr;
	if ( normalInfo != nullptr && normalInfo->normals != nullptr && normalInfo->attribSet == AttribSet::Point &&
		 normalInfo->numNormals >= pointCount )
	{
		for ( int i = 0; i < pointCount; ++i )
		{
			output->setNormal( normalInfo->normals[i], i );
		}
	}

	// Texture coordinates pass through (point attribute; this node keeps the
	// input's shared-point topology, so vertex-level attribs cannot be
	// represented — put the Set Joint before any Facet/uv-unwrap in the chain).
	const SOP_TextureInfo* texInfo = input->getTextures();
	if ( texInfo != nullptr && texInfo->textures != nullptr && texInfo->numTextureLayers > 0 &&
		 texInfo->attribSet == AttribSet::Point && texInfo->numTextures >= pointCount && pointCount > 0 )
	{
		output->setTexCoords( texInfo->textures, pointCount, texInfo->numTextureLayers, 0 );
	}

	for ( int primIndex = 0; primIndex < input->getNumPrimitives(); ++primIndex )
	{
		const SOP_PrimitiveInfo& prim = input->getPrimitive( primIndex );
		if ( prim.type != PrimitiveType::Polygon || prim.numVertices < 3 )
		{
			continue;
		}
		for ( int v = 1; v < prim.numVertices - 1; ++v )
		{
			output->addTriangle( prim.pointIndices[0], prim.pointIndices[v], prim.pointIndices[v + 1] );
		}
	}

	if ( myEnabled && showAnchor && pointCount > 0 )
	{
		int nearest = 0;
		float best = 1e30f;
		for ( int i = 0; i < pointCount; ++i )
		{
			float dx = points[i].x - worldPivot[0];
			float dy = points[i].y - worldPivot[1];
			float dz = points[i].z - worldPivot[2];
			float d2 = dx * dx + dy * dy + dz * dz;
			if ( d2 < best )
			{
				best = d2;
				nearest = i;
			}
		}
		Color markerColor( (float)cr, (float)cg, (float)cb, 1.0f );
		output->setColor( markerColor, nearest );
	}

	setBoundsFromMinMax( output, minv, maxv );

	// Encode the pivot relative to reference points of the geometry itself:
	// worldPivot = p0 + a*(p1-p0) + b*(p2-p0) + c*(p3-p0). Downstream SOPs
	// (Transform etc.) move the points, and the Body SOP rebuilds the pivot
	// from the transformed points, so the joint follows any affine transform
	// applied between this node and the Body SOP.
	int refIdx[4] = { -1, -1, -1, -1 };
	float refW[3] = { 0.0f, 0.0f, 0.0f };
	int refCount = chooseRefPoints( points, pointCount, centroid, refIdx );
	if ( refCount >= 3 )
	{
		const Position& p0 = points[refIdx[0]];
		float e1[3], e2[3], e3[3];
		refEdge( points[refIdx[1]], p0, e1 );
		refEdge( points[refIdx[2]], p0, e2 );
		if ( refCount == 4 )
		{
			refEdge( points[refIdx[3]], p0, e3 );
		}
		else
		{
			// Planar geometry: use the (normalized) plane normal as third axis.
			refCross( e1, e2, e3 );
			float len = sqrtf( refDot( e3, e3 ) );
			if ( len > 1e-12f )
			{
				e3[0] /= len;
				e3[1] /= len;
				e3[2] /= len;
			}
		}

		float d[3] = { worldPivot[0] - p0.x, worldPivot[1] - p0.y, worldPivot[2] - p0.z };
		if ( !solveRefCoords( e1, e2, e3, d, refW ) )
		{
			refIdx[1] = refIdx[2] = refIdx[3] = -1;
			refW[0] = d[0];
			refW[1] = d[1];
			refW[2] = d[2];
		}
	}
	else if ( refCount >= 1 )
	{
		// Degenerate geometry (point/line): plain offset from p0, at least
		// translation-proof.
		const Position& p0 = points[refIdx[0]];
		refW[0] = worldPivot[0] - p0.x;
		refW[1] = worldPivot[1] - p0.y;
		refW[2] = worldPivot[2] - p0.z;
	}

	const int totalPoints = pointCount;
	if ( totalPoints <= 0 )
	{
		// No points: nothing to attach attributes to (empty data pointers
		// must never reach setCustomAttribute).
		return;
	}
	std::vector<int32_t> enabledData( totalPoints, myEnabled ? 1 : 0 );
	std::vector<float> pivotData( totalPoints * 3, 0.0f );
	std::vector<int32_t> pivotSpaceData( totalPoints, 0 ); // 0 = input/object space
	std::vector<int32_t> refData( totalPoints * 4, -1 );
	std::vector<float> refWData( totalPoints * 3, 0.0f );
	for ( int i = 0; i < totalPoints; ++i )
	{
		pivotData[i * 3 + 0] = worldPivot[0];
		pivotData[i * 3 + 1] = worldPivot[1];
		pivotData[i * 3 + 2] = worldPivot[2];
		refData[i * 4 + 0] = refIdx[0];
		refData[i * 4 + 1] = refIdx[1];
		refData[i * 4 + 2] = refIdx[2];
		refData[i * 4 + 3] = refIdx[3];
		refWData[i * 3 + 0] = refW[0];
		refWData[i * 3 + 1] = refW[1];
		refWData[i * 3 + 2] = refW[2];
	}

	SOP_CustomAttribData enabledAttr( kAttrJointEnabled, 1, AttribType::Int );
	enabledAttr.intData = enabledData.data();
	output->setCustomAttribute( &enabledAttr, totalPoints );

	SOP_CustomAttribData pivotAttr( kAttrJointPivot, 3, AttribType::Float );
	pivotAttr.floatData = pivotData.data();
	output->setCustomAttribute( &pivotAttr, totalPoints );

	SOP_CustomAttribData pivotSpaceAttr( kAttrJointPivotSpace, 1, AttribType::Int );
	pivotSpaceAttr.intData = pivotSpaceData.data();
	output->setCustomAttribute( &pivotSpaceAttr, totalPoints );

	SOP_CustomAttribData refAttr( kAttrJointRef, 4, AttribType::Int );
	refAttr.intData = refData.data();
	output->setCustomAttribute( &refAttr, totalPoints );

	SOP_CustomAttribData refWAttr( kAttrJointRefW, 3, AttribType::Float );
	refWAttr.floatData = refWData.data();
	output->setCustomAttribute( &refWAttr, totalPoints );
}

void Box3DSetJointSOP::executeVBO( SOP_VBOOutput*, const OP_Inputs*, void* )
{
}

int32_t Box3DSetJointSOP::getNumInfoCHOPChans( void* )
{
	return 4;
}

void Box3DSetJointSOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	static const char* names[4] = { "enabled", "pivotx", "pivoty", "pivotz" };
	chan->name->setString( names[index] );
	if ( index == 0 )
	{
		chan->value = myEnabled ? 1.0f : 0.0f;
	}
	else
	{
		chan->value = myPivot[index - 1];
	}
}

void Box3DSetJointSOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_NumericParameter p;
		p.name = EnableName;
		p.label = "Enable";
		p.page = "Joint";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_StringParameter p;
		p.name = AnchorModeName;
		p.label = "Anchor Mode";
		p.page = "Joint";
		p.defaultValue = "center";
		const char* names[] = { "center", "tipxp", "tipxn", "tipyp", "tipyn", "tipzp", "tipzn", "custom" };
		const char* labels[] = { "Center", "Tip X+", "Tip X-", "Tip Y+", "Tip Y-", "Tip Z+", "Tip Z-", "Custom" };
		manager->appendMenu( p, 8, names, labels );
	}

	{
		OP_NumericParameter p;
		p.name = AnchorName;
		p.label = "Anchor";
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
		p.name = ShowAnchorName;
		p.label = "Show Anchor";
		p.page = "Joint";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = AnchorcolorName;
		p.label = "Anchor Color";
		p.page = "Joint";
		p.defaultValues[0] = 1.0;
		p.defaultValues[1] = 0.35;
		p.defaultValues[2] = 0.1;
		for ( int i = 0; i < 3; ++i )
		{
			p.minValues[i] = 0.0;
			p.maxValues[i] = 1.0;
			p.minSliders[i] = 0.0;
			p.maxSliders[i] = 1.0;
			p.clampMins[i] = true;
			p.clampMaxes[i] = true;
		}
		manager->appendXYZ( p );
	}
}

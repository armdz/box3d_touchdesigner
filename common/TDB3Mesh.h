// Shared triangulation helper for collision meshes (header-only).
//
// box3d mesh collision is ONE-SIDED: contacts approaching a triangle from
// behind its plane are culled (box3d/src/triangle_manifold.c, "Cull back
// side collision"), and the plane normal comes from the vertex winding.
// TouchDesigner does not guarantee any particular winding — it shades with
// normal attributes and ignores winding — so a Torus/Sphere/inverted surface
// fed straight into a mesh collider may simply not collide.
//
// extractOrientedTriangles() fan-triangulates every polygon primitive and
// flips each triangle so its geometric normal agrees with the geometry's
// normal attribute (point or vertex) when one exists: the side the geometry
// SHADES toward is the side that collides. Without normals the winding is
// passed through unchanged.

#pragma once

#include "CPlusPlus_Common.h"

#include <vector>

namespace tdb3
{

inline void extractOrientedTriangles( const TD::OP_SOPInput* sop, std::vector<int32_t>& indices )
{
	using namespace TD;

	if ( sop == nullptr )
	{
		return;
	}

	const int pointCount = sop->getNumPoints();
	const Position* points = sop->getPointPositions();

	const SOP_NormalInfo* normalInfo = sop->hasNormals() ? sop->getNormals() : nullptr;
	const Vector* normals = normalInfo != nullptr ? normalInfo->normals : nullptr;
	const bool pointNormals =
		normals != nullptr && normalInfo->attribSet == AttribSet::Point && normalInfo->numNormals >= pointCount;
	const bool vertexNormals = normals != nullptr && normalInfo->attribSet == AttribSet::Vertex &&
							  normalInfo->numNormals >= sop->getNumVertices();

	const int primCount = sop->getNumPrimitives();
	for ( int i = 0; i < primCount; ++i )
	{
		const SOP_PrimitiveInfo& prim = sop->getPrimitive( i );
		if ( prim.type != PrimitiveType::Polygon || prim.numVertices < 3 )
		{
			continue;
		}

		for ( int v = 1; v + 1 < prim.numVertices; ++v )
		{
			const int32_t ia = prim.pointIndices[0];
			const int32_t ib = prim.pointIndices[v];
			const int32_t ic = prim.pointIndices[v + 1];
			if ( ia >= pointCount || ib >= pointCount || ic >= pointCount )
			{
				continue;
			}

			bool flip = false;
			float nx = 0.0f, ny = 0.0f, nz = 0.0f;
			bool haveNormal = false;
			if ( pointNormals )
			{
				nx = normals[ia].x + normals[ib].x + normals[ic].x;
				ny = normals[ia].y + normals[ib].y + normals[ic].y;
				nz = normals[ia].z + normals[ib].z + normals[ic].z;
				haveNormal = true;
			}
			else if ( vertexNormals )
			{
				const int32_t va = prim.pointIndicesOffset;
				const int32_t vb = prim.pointIndicesOffset + v;
				const int32_t vc = prim.pointIndicesOffset + v + 1;
				nx = normals[va].x + normals[vb].x + normals[vc].x;
				ny = normals[va].y + normals[vb].y + normals[vc].y;
				nz = normals[va].z + normals[vb].z + normals[vc].z;
				haveNormal = true;
			}

			if ( haveNormal )
			{
				const Position& a = points[ia];
				const Position& b = points[ib];
				const Position& c = points[ic];
				const float e1x = b.x - a.x, e1y = b.y - a.y, e1z = b.z - a.z;
				const float e2x = c.x - a.x, e2y = c.y - a.y, e2z = c.z - a.z;
				const float gx = e1y * e2z - e1z * e2y;
				const float gy = e1z * e2x - e1x * e2z;
				const float gz = e1x * e2y - e1y * e2x;
				flip = ( gx * nx + gy * ny + gz * nz ) < 0.0f;
			}

			indices.push_back( ia );
			if ( flip )
			{
				indices.push_back( ic );
				indices.push_back( ib );
			}
			else
			{
				indices.push_back( ib );
				indices.push_back( ic );
			}
		}
	}
}

} // namespace tdb3

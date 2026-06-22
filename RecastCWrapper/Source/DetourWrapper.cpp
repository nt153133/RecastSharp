#include "DetourWrapper.h"
#include "DetourAlloc.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"

#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include <iostream>
#include <cstring>

#define MAX_PATH_LENGTH         740//74
#define MAX_POINT_PATH_LENGTH   740//74

#define SMOOTH_PATH_STEP_SIZE   8.0f
#define SMOOTH_PATH_SLOP        0.4f

#define VERTEX_SIZE       3
#define INVALID_POLYREF   0

EXPORT_API bool dtwCreateNavMeshData(dtwNavMeshCreateParams* params, unsigned char** outData, int* outDataSize) {
	dtNavMeshCreateParams* dtParams = (dtNavMeshCreateParams*)params;
	return dtCreateNavMeshData(dtParams, outData, outDataSize);
}

EXPORT_API dtwNavMesh* dtwAllocNavMesh() {
	return (dtwNavMesh*)dtAllocNavMesh();
}

EXPORT_API dtwStatus dtwNavMesh_Init(dtwNavMesh* navmesh, dtwNavMeshParams* params) {
	auto dtNm = (dtNavMesh*)navmesh;
	auto dtParams = (dtNavMeshParams*)params;
					
	return (dtwStatus)dtNm->init(dtParams);
}

EXPORT_API dtwStatus dtwNavMesh_AddTile(dtwNavMesh* navmesh, unsigned char* data, int dataSize, int flags, dtwTileRef lastRef, dtwTileRef* result) {
	auto dtNm = (dtNavMesh*)navmesh;
	auto dtLastRef = (dtTileRef)lastRef;
	auto dtResult = (dtTileRef*)result;
	return (dtwStatus)dtNm->addTile(data, dataSize, flags, dtLastRef, dtResult);
}

EXPORT_API dtwStatus dtwNavMesh_RemoveTile(dtwNavMesh* navmesh, dtwTileRef ref, unsigned char** data, int* dataSize) {
	auto dtNm = (dtNavMesh*)navmesh;
	auto dtRef = (dtTileRef)ref;
	return (dtwStatus)dtNm->removeTile(dtRef, data, dataSize);
}

EXPORT_API dtwTileRef dtwNavMesh_GetTileRefAt(dtwNavMesh* navmesh, int x, int y, int layer) {
	auto dtNm = (dtNavMesh*)navmesh;
	return (dtwTileRef)dtNm->getTileRefAt(x, y, layer);
}


EXPORT_API dtwPolyRef* dtwAllocPolyRef() {
	return (dtwPolyRef*)new dtPolyRef();
}

EXPORT_API dtwQueryFilter* dtwAllocQueryFilter() {
	auto filter = new dtQueryFilter();
	return (dtwQueryFilter*)filter;
}

EXPORT_API dtwNavMeshQuery* dtwAllocNavMeshQuery() {
	return (dtwNavMeshQuery*)dtAllocNavMeshQuery();
}

EXPORT_API dtwStatus dtwNavMeshQuery_Init(dtwNavMeshQuery* query, dtwNavMesh* navmesh, int maxNodes) {
	auto dtNm = (dtNavMesh*)navmesh;
	auto dtQuery = (dtNavMeshQuery*)query;

	return (dtwStatus)dtQuery->init(dtNm, maxNodes);
}

EXPORT_API dtwStatus dtwNavMeshQuery_FindNearestPoly(dtwNavMeshQuery* query, const float* center, const float* halfExtents,
	const dtwQueryFilter* filter,
	dtwPolyRef* nearestRef, float* nearestPt) {
	auto dtQuery = (dtNavMeshQuery*)query;
	auto dtFilter = (dtQueryFilter*)filter;
	auto dtNearestRef = (dtPolyRef*)nearestRef;
	return dtQuery->findNearestPoly(center, halfExtents, dtFilter, dtNearestRef, nearestPt);
}

EXPORT_API dtwStatus dtwNavMeshQuery_closestPointOnPoly(dtwNavMeshQuery* query, int nearestRef, const float* pos, float* closest,
	bool* posOverPoly) {
	auto dtQuery = (dtNavMeshQuery*)query;
	return dtQuery->closestPointOnPoly(nearestRef, pos, closest, posOverPoly);
}

EXPORT_API dtwStatus dtwNavMeshQuery_closestPointOnPolyBoundary(dtwNavMeshQuery* query, int nearestRef, const float* pos, float* closest) {
	auto dtQuery = (dtNavMeshQuery*)query;
	return dtQuery->closestPointOnPolyBoundary(nearestRef, pos, closest);
}

EXPORT_API dtwStatus dtwNavMeshQuery_FindPath(dtwNavMeshQuery* query, 
	dtwPolyRef startRef, dtwPolyRef endRef, 
	const float* startPos, const float* endPos, 
	const dtwQueryFilter* filter, dtwPolyRef* path, int* pathCount, const int maxPath) {
	auto dtQuery = (dtNavMeshQuery*)query;
	auto dtStartRef = (dtPolyRef)startRef;
	auto dtEndRef = (dtPolyRef)endRef;
	auto dtFilter = (dtQueryFilter*)filter;
	auto dtPath = (dtPolyRef*)path;

	return dtQuery->findPath(dtStartRef, dtEndRef, startPos, endPos, dtFilter, dtPath, pathCount, maxPath);
}

EXPORT_API dtwStatus dtwNavMeshQuery_FindStraightPath(dtwNavMeshQuery* query, const float* startPos, const float* endPos,
	const dtPolyRef* path, const int pathSize,
	float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	int* straightPathCount, const int maxStraightPath, const int options){
	auto dtQuery = (dtNavMeshQuery*)query;
	auto dtPath = (dtPolyRef*)path;
	auto dtStraightPathRefs = (dtPolyRef*)straightPathRefs;
	return dtQuery->findStraightPath(startPos, endPos, dtPath, pathSize, straightPath, straightPathFlags, dtStraightPathRefs, straightPathCount, maxStraightPath, options);
}

// Allocate with Detour's allocator so tile data added with DT_TILE_FREE_DATA is
// released by the matching dtFree when the navmesh is destroyed.
EXPORT_API void* dtwAlloc(int size) {
	return dtAlloc(size, DT_ALLOC_PERM);
}

EXPORT_API void dtwFree(void* ptr) {
	dtFree(ptr);
}

EXPORT_API void dtwFreeNavMesh(dtwNavMesh* navmesh) {
	dtFreeNavMesh((dtNavMesh*)navmesh);
}

EXPORT_API void dtwFreeNavMeshQuery(dtwNavMeshQuery* query) {
	dtFreeNavMeshQuery((dtNavMeshQuery*)query);
}

EXPORT_API void dtwFreeQueryFilter(dtwQueryFilter* filter) {
	delete (dtQueryFilter*)filter;
}

EXPORT_API void dtwFreePolyRef(dtwPolyRef* polyRef) {
	delete (dtPolyRef*)polyRef;
}

EXPORT_API bool dtwStatusSucceed(dtwStatus status) {
	return dtStatusSucceed(status);
}

EXPORT_API bool dtwStatusFailed(dtwStatus status) {
	return dtStatusFailed(status);
}


EXPORT_API bool dtwStatusInProgress(dtwStatus status) {
	return dtStatusInProgress(status);
}


EXPORT_API bool dtwStatusDetail(dtwStatus status, unsigned int detail) {
	return dtStatusDetail(status, detail);
}

EXPORT_API dtStatus findSmoothPath(dtwNavMeshQuery* query, const dtwQueryFilter* filter, const float* startPos, const float* endPos,
	const dtPolyRef* polyPath, unsigned int polyPathSize,
	float* smoothPath, int* smoothPathSize, unsigned int maxSmoothPathSize)
{
	auto m_navMeshQuery = (dtNavMeshQuery*)query;
	auto dtFilter = (dtQueryFilter*)filter;
	*smoothPathSize = 0;
	unsigned int nsmoothPath = 0;

	dtPolyRef polys[MAX_PATH_LENGTH];
	memcpy(polys, polyPath, sizeof(dtPolyRef)*polyPathSize);
	unsigned int npolys = polyPathSize;

	float iterPos[VERTEX_SIZE], targetPos[VERTEX_SIZE];
	dtStatus dtResult = m_navMeshQuery->closestPointOnPolyBoundary(polys[0], startPos, iterPos);
	if (dtStatusFailed(dtResult))
	{
		return DT_FAILURE;
	}

	dtResult = m_navMeshQuery->closestPointOnPolyBoundary(polys[npolys - 1], endPos, targetPos);
	if (dtStatusFailed(dtResult))
	{
		return DT_FAILURE;
	}

	dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], iterPos);
	++nsmoothPath;

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.
	while (npolys && nsmoothPath < maxSmoothPathSize)
	{
		// Find location to steer towards.
		float steerPos[VERTEX_SIZE];
		unsigned char steerPosFlag;
		dtPolyRef steerPosRef = INVALID_POLYREF;

		if (!getSteerTarget(query, iterPos, targetPos, SMOOTH_PATH_SLOP, polys, npolys, steerPos, steerPosFlag, steerPosRef))
		{
			break;
		}

		bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END);
		bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION);

		// Find movement delta.
		float delta[VERTEX_SIZE];
		dtVsub(delta, steerPos, iterPos);
		float len = dtSqrt(dtVdot(delta, delta));
		// If the steer target is end of path or off-mesh link, do not move past the location.
		if ((endOfPath || offMeshConnection) && len < SMOOTH_PATH_STEP_SIZE)
		{
			len = 1.0f;
		}
		else
		{
			len = SMOOTH_PATH_STEP_SIZE / len;
		}

		float moveTgt[VERTEX_SIZE];
		dtVmad(moveTgt, iterPos, delta, len);

		// Move
		float result[VERTEX_SIZE];
		const static unsigned int MAX_VISIT_POLY = 16;
		dtPolyRef visited[MAX_VISIT_POLY];

		unsigned int nvisited = 0;
		m_navMeshQuery->moveAlongSurface(polys[0], iterPos, moveTgt, dtFilter, result, visited, (int*)&nvisited, MAX_VISIT_POLY);
		npolys = fixupCorridor(polys, npolys, MAX_PATH_LENGTH, visited, nvisited);

		m_navMeshQuery->getPolyHeight(polys[0], result, &result[1]);
		result[1] += 0.5f;
		dtVcopy(iterPos, result);

		// Handle end of path and off-mesh links when close enough.
		if (endOfPath && inRangeYZX(iterPos, steerPos, SMOOTH_PATH_SLOP, 1.0f))
		{
			// Reached end of path.
			dtVcopy(iterPos, targetPos);
			if (nsmoothPath < maxSmoothPathSize)
			{
				dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], iterPos);
				++nsmoothPath;
			}
			break;
		}
		/*
		else if (offMeshConnection && inRangeYZX(iterPos, steerPos, SMOOTH_PATH_SLOP, 1.0f))
		{
			// Advance the path up to and over the off-mesh connection.
			dtPolyRef prevRef = INVALID_POLYREF;
			dtPolyRef polyRef = polys[0];
			unsigned int npos = 0;
			while (npos < npolys && polyRef != steerPosRef)
			{
				prevRef = polyRef;
				polyRef = polys[npos];
				++npos;
			}

			for (unsigned int i = npos; i < npolys; ++i)
			{
				polys[i - npos] = polys[i];
			}

			npolys -= npos;

			// Handle the connection.
			float startPos[VERTEX_SIZE], endPos[VERTEX_SIZE];
			dtResult = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
			if (dtStatusSucceed(dtResult))
			{
				if (nsmoothPath < maxSmoothPathSize)
				{
					dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], startPos);
					++nsmoothPath;
				}
				// Move position at the other side of the off-mesh link.
				dtVcopy(iterPos, endPos);

				m_navMeshQuery->getPolyHeight(polys[0], iterPos, &iterPos[1]);
				iterPos[1] += 0.5f;
			}
		}
		*/
		// Store results.
		if (nsmoothPath < maxSmoothPathSize)
		{
			dtVcopy(&smoothPath[nsmoothPath * VERTEX_SIZE], iterPos);
			++nsmoothPath;
		}
	}

	*smoothPathSize = nsmoothPath;

	// this is most likely a loop
	return nsmoothPath < MAX_POINT_PATH_LENGTH ? DT_SUCCESS : DT_FAILURE;
}

EXPORT_API bool getSteerTarget(dtwNavMeshQuery* query, const float* startPos, const float* endPos,
	float minTargetDist, const dtPolyRef* path, unsigned int pathSize,
	float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef)
{
	auto m_navMeshQuery = (dtNavMeshQuery*)query;
	// Find steer target.
	static const unsigned int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS * VERTEX_SIZE];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	unsigned int nsteerPath = 0;
	dtStatus dtResult = m_navMeshQuery->findStraightPath(startPos, endPos, path, pathSize,
		steerPath, steerPathFlags, steerPathPolys, (int*)&nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath || dtStatusFailed(dtResult))
	{
		return false;
	}

	// Find vertex far enough to steer to.
	unsigned int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRangeYZX(&steerPath[ns * VERTEX_SIZE], startPos, minTargetDist, 1000.0f))
		{
			break;
		}
		++ns;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
	{
		return false;
	}

	dtVcopy(steerPos, &steerPath[ns * VERTEX_SIZE]);
	steerPos[1] = startPos[1];  // keep Z value
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];

	return true;
}

EXPORT_API unsigned int fixupCorridor(dtPolyRef* path, unsigned int npath, unsigned int maxPath,
	const dtPolyRef* visited, unsigned int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;

	// Find furthest common polygon.
	for (int i = npath - 1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited - 1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
		{
			break;
		}
	}

	// If no intersection found just return current path.
	if (furthestPath == -1 || furthestVisited == -1)
	{
		return npath;
	}

	// Concatenate paths.

	// Adjust beginning of the buffer to include the visited.
	unsigned int req = nvisited - furthestVisited;
	unsigned int orig = unsigned int(furthestPath + 1) < npath ? furthestPath + 1 : npath;
	unsigned int size = npath > orig ? npath - orig : 0;
	if (req + size > maxPath)
	{
		size = maxPath - req;
	}

	if (size)
	{
		memmove(path + req, path + orig, size * sizeof(dtPolyRef));
	}

	// Store visited
	for (unsigned int i = 0; i < req; ++i)
	{
		path[i] = visited[(nvisited - 1) - i];
	}

	return req + size;
}

EXPORT_API bool inRangeYZX(const float* v1, const float* v2, float r, float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1]; // elevation
	const float dz = v2[2] - v1[2];
	return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
}

EXPORT_API float dtSqrt(float x)
{
	return dtMathSqrtf(x);
}

// Turns a built Recast poly mesh + detail mesh into a Detour tile blob (the bytes a navmesh
// addTile / a NavMeshSet ".nav" file stores). Encapsulates the area->flag assignment and the
// dtNavMeshCreateParams plumbing so the managed side never has to marshal the mesh arrays.
// Every poly in a built poly mesh is walkable, so they all get a single WALK flag (0x01).
// outData is allocated by Detour (dtAlloc); free it with dtwFree after copying it out.
EXPORT_API bool rcwCreateNavMeshTileData(void* polyMesh, void* polyMeshDetail,
	float walkableHeight, float walkableRadius, float walkableClimb,
	float cs, float ch, int tileX, int tileY, bool buildBvTree,
	unsigned char** outData, int* outDataSize)
{
	auto pmesh = (rcPolyMesh*)polyMesh;
	auto dmesh = (rcPolyMeshDetail*)polyMeshDetail;

	for (int i = 0; i < pmesh->npolys; ++i)
	{
		if (pmesh->areas[i] == RC_WALKABLE_AREA)
			pmesh->areas[i] = 0;          // SAMPLE_POLYAREA_GROUND
		pmesh->flags[i] = 1;              // SAMPLE_POLYFLAGS_WALK
	}

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = pmesh->verts;
	params.vertCount = pmesh->nverts;
	params.polys = pmesh->polys;
	params.polyAreas = pmesh->areas;
	params.polyFlags = pmesh->flags;
	params.polyCount = pmesh->npolys;
	params.nvp = pmesh->nvp;
	params.detailMeshes = dmesh->meshes;
	params.detailVerts = dmesh->verts;
	params.detailVertsCount = dmesh->nverts;
	params.detailTris = dmesh->tris;
	params.detailTriCount = dmesh->ntris;
	params.walkableHeight = walkableHeight;
	params.walkableRadius = walkableRadius;
	params.walkableClimb = walkableClimb;
	params.tileX = tileX;
	params.tileY = tileY;
	params.tileLayer = 0;
	rcVcopy(params.bmin, pmesh->bmin);
	rcVcopy(params.bmax, pmesh->bmax);
	params.cs = cs;
	params.ch = ch;
	params.buildBvTree = buildBvTree;

	return dtCreateNavMeshData(&params, outData, outDataSize);
}

// Flattens a Recast detail mesh into a triangle-soup of world-space vertices (9 floats per
// triangle: 3 verts x xyz), resolving each sub-mesh's base offsets. Lets callers export the
// actual walkable navmesh surface as geometry. outVerts is dtAlloc'd; free it with dtwFree.
EXPORT_API bool rcwGetPolyMeshDetailTriVerts(void* polyMeshDetail, float** outVerts, int* outFloatCount)
{
	auto dmesh = (rcPolyMeshDetail*)polyMeshDetail;
	*outVerts = nullptr;
	*outFloatCount = 0;
	if (!dmesh || dmesh->ntris == 0)
		return true;

	float* out = (float*)dtAlloc(sizeof(float) * dmesh->ntris * 9, DT_ALLOC_PERM);
	if (!out)
		return false;

	int o = 0;
	for (int m = 0; m < dmesh->nmeshes; ++m)
	{
		const unsigned int bverts = dmesh->meshes[m * 4 + 0];
		const unsigned int btris = dmesh->meshes[m * 4 + 2];
		const unsigned int ntris = dmesh->meshes[m * 4 + 3];
		const float* verts = &dmesh->verts[bverts * 3];
		const unsigned char* tris = &dmesh->tris[btris * 4];
		for (unsigned int t = 0; t < ntris; ++t)
			for (int k = 0; k < 3; ++k)
			{
				const float* v = &verts[tris[t * 4 + k] * 3];
				out[o++] = v[0];
				out[o++] = v[1];
				out[o++] = v[2];
			}
	}

	*outVerts = out;
	*outFloatCount = o;
	return true;
}

// Flattens a loaded Detour navmesh's walkable surface into a world-space triangle soup (9 floats
// per triangle) by walking every tile's per-poly detail mesh. Lets a viewer render a .nav directly.
// outVerts is dtAlloc'd; free it with dtwFree.
EXPORT_API bool dtwGetNavMeshTriVerts(dtwNavMesh* navmeshHandle, float** outVerts, int* outFloatCount)
{
	auto mesh = (const dtNavMesh*)navmeshHandle;
	*outVerts = nullptr;
	*outFloatCount = 0;
	if (!mesh)
		return false;

	int total = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header)
			continue;
		for (int p = 0; p < tile->header->polyCount; ++p)
			total += tile->detailMeshes[p].triCount;
	}
	if (total == 0)
		return true;

	float* out = (float*)dtAlloc(sizeof(float) * total * 9, DT_ALLOC_PERM);
	if (!out)
		return false;

	int o = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header)
			continue;
		for (int p = 0; p < tile->header->polyCount; ++p)
		{
			const dtPoly* poly = &tile->polys[p];
			const dtPolyDetail* pd = &tile->detailMeshes[p];
			for (int j = 0; j < pd->triCount; ++j)
			{
				const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
				for (int k = 0; k < 3; ++k)
				{
					const float* v = t[k] < poly->vertCount
						? &tile->verts[poly->verts[t[k]] * 3]
						: &tile->detailVerts[(pd->vertBase + t[k] - poly->vertCount) * 3];
					out[o++] = v[0];
					out[o++] = v[1];
					out[o++] = v[2];
				}
			}
		}
	}

	*outVerts = out;
	*outFloatCount = o;
	return true;
}
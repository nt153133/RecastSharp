#include "DetourWrapper.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"

#include "DetourNavMeshQuery.h"
#include <iostream>

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

EXPORT_API void dtwFree(void* ptr) {
	dtFree(ptr);
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
#ifndef DETOUR_WRAPPER_H
#define DETOUR_WRAPPER_H

#include "WrapperCommon.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"

EXPORT_API bool dtwCreateNavMeshData(dtwNavMeshCreateParams* params, unsigned char** outData, int* outDataSize);

EXPORT_API dtwNavMesh* dtwAllocNavMesh();
EXPORT_API dtwStatus dtwNavMesh_Init(dtwNavMesh* navmesh, dtwNavMeshParams* params);
EXPORT_API dtwStatus dtwNavMesh_AddTile(dtwNavMesh* navmesh, unsigned char* data, int dataSize, int flags, dtwTileRef lastRef, dtwTileRef* result);
EXPORT_API dtwStatus dtwNavMesh_RemoveTile(dtwNavMesh* navmesh, dtwTileRef ref, unsigned char** data, int* dataSize);
EXPORT_API dtwTileRef dtwNavMesh_GetTileRefAt(dtwNavMesh* navmesh, int x, int y, int layer);

EXPORT_API dtwPolyRef* dtwAllocPolyRef();
EXPORT_API dtwQueryFilter* dtwAllocQueryFilter();

EXPORT_API dtwNavMeshQuery* dtwAllocNavMeshQuery();
EXPORT_API dtwStatus dtwNavMeshQuery_Init(dtwNavMeshQuery* query, dtwNavMesh* navmesh, int maxNodes);
EXPORT_API dtwStatus dtwNavMeshQuery_FindNearestPoly(dtwNavMeshQuery* query, const float* center, const float* halfExtents,
	const dtwQueryFilter* filter,
	dtwPolyRef* nearestRef, float* nearestPt);
EXPORT_API dtwStatus dtwNavMeshQuery_FindPath(dtwNavMeshQuery* query,
	dtwPolyRef startRef, dtwPolyRef endRef,
	const float* startPos, const float* endPos,
	const dtwQueryFilter* filter, dtwPolyRef* path, int* pathCount, const int maxPath);

EXPORT_API void* dtwAlloc(int size);
EXPORT_API void dtwFree(void* ptr);

// Type-correct deallocators. dtwFree only calls dtFree (no destructor), which leaks
// the internal allocations of a navmesh/query and is undefined behavior for the
// new'd filter/polyref. Use these to release the matching dtwAlloc* handles.
EXPORT_API void dtwFreeNavMesh(dtwNavMesh* navmesh);
EXPORT_API void dtwFreeNavMeshQuery(dtwNavMeshQuery* query);
EXPORT_API void dtwFreeQueryFilter(dtwQueryFilter* filter);
EXPORT_API void dtwFreePolyRef(dtwPolyRef* polyRef);


EXPORT_API bool dtwStatusSucceed(dtwStatus status);
EXPORT_API bool dtwStatusFailed(dtwStatus status);
EXPORT_API bool dtwStatusInProgress(dtwStatus status);
EXPORT_API bool dtwStatusDetail(dtwStatus status, unsigned int detail);

EXPORT_API bool getSteerTarget(dtwNavMeshQuery* query, const float* startPos, const float* endPos,
	float minTargetDist, const dtPolyRef* path, unsigned int pathSize,
	float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef);

EXPORT_API float dtSqrt(float x);

// Build a Detour tile blob from a Recast poly mesh + detail mesh (see DetourWrapper.cpp).
EXPORT_API bool rcwCreateNavMeshTileData(void* polyMesh, void* polyMeshDetail,
	float walkableHeight, float walkableRadius, float walkableClimb,
	float cs, float ch, int tileX, int tileY, bool buildBvTree,
	unsigned char** outData, int* outDataSize);

// Flatten a detail mesh into a world-space triangle-soup (9 floats/triangle); free with dtwFree.
EXPORT_API bool rcwGetPolyMeshDetailTriVerts(void* polyMeshDetail, float** outVerts, int* outFloatCount);

// Flatten a loaded navmesh's walkable surface into a world-space triangle-soup; free with dtwFree.
EXPORT_API bool dtwGetNavMeshTriVerts(dtwNavMesh* navmesh, float** outVerts, int* outFloatCount);
EXPORT_API bool inRangeYZX(const float* v1, const float* v2, float r, float h);
EXPORT_API unsigned int fixupCorridor(dtPolyRef* path, unsigned int npath, unsigned int maxPath,
	const dtPolyRef* visited, unsigned int nvisited);

#endif
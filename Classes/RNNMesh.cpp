//
//  RNNMesh.cpp
//  rayne-navigation
//
//  Copyright 2014 by Überpixel. All rights reserved.
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
//  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include "RNNMesh.h"

namespace RN
{
	namespace navigation
	{
		RNDefineMeta(Mesh, RN::Object)
		
		BuildContext::BuildContext()
		{
			
		}
		
		void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
		{
			RN_ASSERT(category != RC_LOG_ERROR, msg);
			RNDebug(msg);
		}
		
		Mesh::Mesh()
		{
			Initialize();
		}
		
		Mesh::Mesh(RN::Model *model)
		{
			Initialize();
			GenerateFromModel(model);
		}
		
		Mesh::~Mesh()
		{
			Cleanup();
		}
		
		void Mesh::Initialize()
		{
			_polyMesh = nullptr;
			_polyMeshDetail = nullptr;
			_navigationMesh = nullptr;
			
			_cellSize = 0.3f;
			_cellHeight = 0.2f;
			_agentHeight = 2.0f;
			_agentRadius = 0.6f;
			_agentMaxClimb = 0.9f;
			_agentMaxSlope = 45.0f;
			_regionMinSize = 8;
			_regionMergeSize = 20;
			_edgeMaxLen = 12.0f;
			_edgeMaxError = 1.3f;
			_vertsPerPoly = 6.0f;
			_detailSampleDist = 6.0f;
			_detailSampleMaxError = 1.0f;
			_partitionType = Watershed;
		}
		
		bool Mesh::GenerateFromModel(RN::Model *model)
		{
			RN::Array *models = new RN::Array();
			models->AddObject(model);
			return GenerateFromModels(models->Autorelease());
		}
		
		bool Mesh::GenerateFromModels(RN::Array *models)
		{
			RN_ASSERT(models && models->GetCount(), "There must be at least one model.");
			
			Cleanup();
			
			int32 numberOfVertices = 0;
			int32 numberOfTriangles = 0;
			models->Enumerate<RN::Model>([&](RN::Model *model, size_t index, bool stop){
				for(int i = 0; i < model->GetMeshCount(0); i++)
				{
					numberOfVertices += model->GetMeshAtIndex(0, i)->GetVerticesCount();
					numberOfTriangles += model->GetMeshAtIndex(0, i)->GetIndicesCount()/3;
				}
			});
			
			//
			// Step 1. Initialize build config.
			//
			
			// Init build configuration from GUI
			memset(&_recastConfig, 0, sizeof(_recastConfig));
			_recastConfig.cs = _cellSize;
			_recastConfig.ch = _cellHeight;
			_recastConfig.walkableSlopeAngle = _agentMaxSlope;
			_recastConfig.walkableHeight = (int)ceilf(_agentHeight / _recastConfig.ch);
			_recastConfig.walkableClimb = (int)floorf(_agentMaxClimb / _recastConfig.ch);
			_recastConfig.walkableRadius = (int)ceilf(_agentRadius / _recastConfig.cs);
			_recastConfig.maxEdgeLen = (int)(_edgeMaxLen / _cellSize);
			_recastConfig.maxSimplificationError = _edgeMaxError;
			_recastConfig.minRegionArea = (int)rcSqr(_regionMinSize);		// Note: area = size*size
			_recastConfig.mergeRegionArea = (int)rcSqr(_regionMergeSize);	// Note: area = size*size
			_recastConfig.maxVertsPerPoly = (int)_vertsPerPoly;
			_recastConfig.detailSampleDist = _detailSampleDist < 0.9f ? 0 : _cellSize * _detailSampleDist;
			_recastConfig.detailSampleMaxError = _cellHeight * _detailSampleMaxError;
			
			// Set the area where the navigation will be build.
			// Here the bounds of the input mesh are used, but the
			// area could be specified by an user defined box, etc.
			RN::AABB boundingBox;
			models->Enumerate<RN::Model>([&](RN::Model *model, size_t index, bool stop){
				boundingBox += model->GetBoundingBox();
			});
			rcVcopy(_recastConfig.bmin, &boundingBox.minExtend.x);
			rcVcopy(_recastConfig.bmax, &boundingBox.maxExtend.x);
			rcCalcGridSize(_recastConfig.bmin, _recastConfig.bmax, _recastConfig.cs, &_recastConfig.width, &_recastConfig.height);
			
			//Create build context
			BuildContext *buildContext = new BuildContext;
			
			// Reset build times gathering.
			buildContext->resetTimers();
			
			// Start the build process.
			buildContext->startTimer(RC_TIMER_TOTAL);
			
			buildContext->log(RC_LOG_PROGRESS, "Building navigation:");
			buildContext->log(RC_LOG_PROGRESS, " - %d x %d cells", _recastConfig.width, _recastConfig.height);
			buildContext->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", numberOfVertices/1000.0f, numberOfTriangles/1000.0f);
			
			//
			// Step 2. Rasterize input polygon soup.
			//
			
			// Allocate voxel heightfield where we rasterize our input data to.
			rcHeightfield *heightfield = rcAllocHeightfield();
			if(!heightfield)
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
				return false;
			}
			if(!rcCreateHeightfield(buildContext, *heightfield, _recastConfig.width, _recastConfig.height, _recastConfig.bmin, _recastConfig.bmax, _recastConfig.cs, _recastConfig.ch))
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
				return false;
			}
			
			// Allocate array that can hold triangle area types.
			// If you have multiple meshes you need to process, allocate
			// an array which can hold the max number of triangles you need to process.
			unsigned char* triangleAreas = new unsigned char[numberOfTriangles];
			if(!triangleAreas)
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'triangleAreas' (%d).", numberOfTriangles);
				return false;
			}
			
			// Find triangles which are walkable based on their slope and rasterize them.
			// If your input data is multiple meshes, you can transform them here, calculate
			// the area type for each of the meshes and rasterize them.
			memset(triangleAreas, 0, numberOfTriangles*sizeof(unsigned char));
			
			models->Enumerate<RN::Model>([&](RN::Model *model, size_t index, bool stop){
				for(int i = 0; i < model->GetMeshCount(0); i++)
				{
					RN::Mesh *mesh = model->GetMeshAtIndex(0, i);
					
					
					const MeshDescriptor *posdescriptor = mesh->GetDescriptorForFeature(MeshFeature::Vertices);
					const MeshDescriptor *inddescriptor = mesh->GetDescriptorForFeature(MeshFeature::Indices);
					const uint8 *pospointer = mesh->GetVerticesData<uint8>() + posdescriptor->offset;
					
					const Vector3 *vertex;
					float *vertices = new float[mesh->GetVerticesCount()*3];
					int32 *indices = new int32[mesh->GetIndicesCount()];
					size_t stride = mesh->GetStride();
					
					for(int n = 0; n < mesh->GetVerticesCount(); n++)
					{
						vertex = reinterpret_cast<const Vector3 *>(pospointer + stride * n);
						vertices[n*3+0] = vertex->x;
						vertices[n*3+1] = vertex->y;
						vertices[n*3+2] = vertex->z;
					}
					
					switch(inddescriptor->elementSize)
					{
						case 1:
						{
							const uint8 *index = mesh->GetIndicesData<uint8>();
							for(int n = 0; n < mesh->GetIndicesCount(); n++)
							{
								indices[n] = (*index ++);
							}
							
							break;
						}
							
						case 2:
						{
							const uint16 *index = mesh->GetIndicesData<uint16>();
							for(int n = 0; n < mesh->GetIndicesCount(); n++)
							{
								indices[n] = (*index ++);
							}
							
							break;
						}
							
						case 4:
						{
							const uint32 *index = mesh->GetIndicesData<uint32>();
							for(int n = 0; n < mesh->GetIndicesCount(); n++)
							{
								indices[n] = (*index ++);
							}
							
							break;
						}
					}
					
					rcMarkWalkableTriangles(buildContext, _recastConfig.walkableSlopeAngle, vertices, static_cast<int>(mesh->GetVerticesCount()), indices, static_cast<int>(mesh->GetIndicesCount())/3, triangleAreas);
					rcRasterizeTriangles(buildContext, vertices, static_cast<int>(mesh->GetVerticesCount()), indices, triangleAreas, static_cast<int>(mesh->GetIndicesCount())/3, *heightfield, _recastConfig.walkableClimb);
				}
			});
			
			delete [] triangleAreas;
			
			//
			// Step 3. Filter walkables surfaces.
			//
			
			// Once all geoemtry is rasterized, we do initial pass of filtering to
			// remove unwanted overhangs caused by the conservative rasterization
			// as well as filter spans where the character cannot possibly stand.
			rcFilterLowHangingWalkableObstacles(buildContext, _recastConfig.walkableClimb, *heightfield);
			rcFilterLedgeSpans(buildContext, _recastConfig.walkableHeight, _recastConfig.walkableClimb, *heightfield);
			rcFilterWalkableLowHeightSpans(buildContext, _recastConfig.walkableHeight, *heightfield);
			
			
			//
			// Step 4. Partition walkable surface to simple regions.
			//
			
			// Compact the heightfield so that it is faster to handle from now on.
			// This will result more cache coherent data as well as the neighbours
			// between walkable cells will be calculated.
			rcCompactHeightfield *compactHeightfield = rcAllocCompactHeightfield();
			if(!compactHeightfield)
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
				return false;
			}
			if(!rcBuildCompactHeightfield(buildContext, _recastConfig.walkableHeight, _recastConfig.walkableClimb, *heightfield, *compactHeightfield))
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
				return false;
			}
			
			rcFreeHeightField(heightfield);
			
			// Erode the walkable area by agent radius.
			if(!rcErodeWalkableArea(buildContext, _recastConfig.walkableRadius, *compactHeightfield))
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
				return false;
			}
			
			// (Optional) Mark areas.
		/*	const ConvexVolume* vols = m_geom->getConvexVolumes();
			for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
				rcMarkConvexPolyArea(buildContext, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *_compactHeightfield);*/
			
			
			// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
			// There are 3 martitioning methods, each with some pros and cons:
			// 1) Watershed partitioning
			//   - the classic Recast partitioning
			//   - creates the nicest tessellation
			//   - usually slowest
			//   - partitions the heightfield into nice regions without holes or overlaps
			//   - the are some corner cases where this method creates produces holes and overlaps
			//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
			//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
			//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
			// 2) Monotone partioning
			//   - fastest
			//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
			//   - creates long thin polygons, which sometimes causes paths with detours
			//   * use this if you want fast navmesh generation
			// 3) Layer partitoining
			//   - quite fast
			//   - partitions the heighfield into non-overlapping regions
			//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
			//   - produces better triangles than monotone partitioning
			//   - does not have the corner cases of watershed partitioning
			//   - can be slow and create a bit ugly tessellation (still better than monotone)
			//     if you have large open areas with small obstacles (not a problem if you use tiles)
			//   * good choice to use for tiled navmesh with medium and small sized tiles
			
			if(_partitionType == PartitionType::Watershed)
			{
				// Prepare for region partitioning, by calculating distance field along the walkable surface.
				if(!rcBuildDistanceField(buildContext, *compactHeightfield))
				{
					buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
					return false;
				}
				
				// Partition the walkable surface into simple regions without holes.
				if(!rcBuildRegions(buildContext, *compactHeightfield, 0, _recastConfig.minRegionArea, _recastConfig.mergeRegionArea))
				{
					buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
					return false;
				}
			}
			else if(_partitionType == PartitionType::Monotone)
			{
				// Partition the walkable surface into simple regions without holes.
				// Monotone partitioning does not need distancefield.
				if(!rcBuildRegionsMonotone(buildContext, *compactHeightfield, 0, _recastConfig.minRegionArea, _recastConfig.mergeRegionArea))
				{
					buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
					return false;
				}
			}
			else // SAMPLE_PARTITION_LAYERS
			{
				// Partition the walkable surface into simple regions without holes.
				if(!rcBuildLayerRegions(buildContext, *compactHeightfield, 0, _recastConfig.minRegionArea))
				{
					buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
					return false;
				}
			}
			
			//
			// Step 5. Trace and simplify region contours.
			//
			
			// Create contours.
			rcContourSet *contourSet = rcAllocContourSet();
			if(!contourSet)
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory '_contourSet'.");
				return false;
			}
			if(!rcBuildContours(buildContext, *compactHeightfield, _recastConfig.maxSimplificationError, _recastConfig.maxEdgeLen, *contourSet))
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
				return false;
			}
			
			//
			// Step 6. Build polygons mesh from contours.
			//
			
			// Build polygon navmesh from the contours.
			_polyMesh = rcAllocPolyMesh();
			if(!_polyMesh)
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory '_polyMesh'.");
				return false;
			}
			if(!rcBuildPolyMesh(buildContext, *contourSet, _recastConfig.maxVertsPerPoly, *_polyMesh))
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
				return false;
			}
			
			//
			// Step 7. Create detail mesh which allows to access approximate height on each polygon.
			//
			
			_polyMeshDetail = rcAllocPolyMeshDetail();
			if(!_polyMeshDetail)
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
				return false;
			}
			
			if(!rcBuildPolyMeshDetail(buildContext, *_polyMesh, *compactHeightfield, _recastConfig.detailSampleDist, _recastConfig.detailSampleMaxError, *_polyMeshDetail))
			{
				buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
				return false;
			}

			rcFreeCompactHeightfield(compactHeightfield);
			rcFreeContourSet(contourSet);
			
			// At this point the navigation mesh data is ready, you can access it from m_pmesh.
			// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
			
			//
			// (Optional) Step 8. Create Detour data from Recast poly mesh.
			//
			
			// The GUI may allow more max points per polygon than Detour can handle.
			// Only build the detour navmesh if we do not exceed the limit.
			if(_recastConfig.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
			{
				unsigned char* navData = 0;
				int navDataSize = 0;
				
				// Update poly flags from areas.
				for (int i = 0; i < _polyMesh->npolys; ++i)
				{
					_polyMesh->areas[i] = 0;
					_polyMesh->flags[i] = 1;
					
					/*if(_polyMesh->areas[i] == RC_WALKABLE_AREA)
						_polyMesh->areas[i] = SAMPLE_POLYAREA_GROUND;
					
					if(_polyMesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
						_polyMesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
						_polyMesh->areas[i] == SAMPLE_POLYAREA_ROAD)
					{
						_polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
					}
					else if(_polyMesh->areas[i] == SAMPLE_POLYAREA_WATER)
					{
						_polyMesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
					}
					else if(_polyMesh->areas[i] == SAMPLE_POLYAREA_DOOR)
					{
						_polygonMesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
					}*/
				}
				
				
				dtNavMeshCreateParams params;
				memset(&params, 0, sizeof(params));
				params.verts = _polyMesh->verts;
				params.vertCount = _polyMesh->nverts;
				params.polys = _polyMesh->polys;
				params.polyAreas = _polyMesh->areas;
				params.polyFlags = _polyMesh->flags;
				params.polyCount = _polyMesh->npolys;
				params.nvp = _polyMesh->nvp;
				params.detailMeshes = _polyMeshDetail->meshes;
				params.detailVerts = _polyMeshDetail->verts;
				params.detailVertsCount = _polyMeshDetail->nverts;
				params.detailTris = _polyMeshDetail->tris;
				params.detailTriCount = _polyMeshDetail->ntris;
/*				params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
				params.offMeshConRad = m_geom->getOffMeshConnectionRads();
				params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
				params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
				params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
				params.offMeshConUserID = m_geom->getOffMeshConnectionId();
				params.offMeshConCount = m_geom->getOffMeshConnectionCount();*/
				params.walkableHeight = _agentHeight;
				params.walkableRadius = _agentRadius;
				params.walkableClimb = _agentMaxClimb;
				rcVcopy(params.bmin, _polyMesh->bmin);
				rcVcopy(params.bmax, _polyMesh->bmax);
				params.cs = _recastConfig.cs;
				params.ch = _recastConfig.ch;
				params.buildBvTree = true;
				
				if(!dtCreateNavMeshData(&params, &navData, &navDataSize))
				{
					buildContext->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
					return false;
				}
				
				_navigationMesh = dtAllocNavMesh();
				if(!_navigationMesh)
				{
					dtFree(navData);
					buildContext->log(RC_LOG_ERROR, "Could not create Detour navmesh");
					return false;
				}
				
				dtStatus status;
				
				status = _navigationMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
				if(dtStatusFailed(status))
				{
					dtFree(navData);
					buildContext->log(RC_LOG_ERROR, "Could not init Detour navmesh");
					return false;
				}
			}
			
			buildContext->stopTimer(RC_TIMER_TOTAL);
			
			// Show performance stats.
			buildContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", _polyMesh->nverts, _polyMesh->npolys);
			
			delete buildContext;
			
			return true;
		}
		
		void Mesh::Cleanup()
		{
			rcFreePolyMesh(_polyMesh);
			_polyMesh = 0;
			rcFreePolyMeshDetail(_polyMeshDetail);
			_polyMeshDetail = 0;
			dtFreeNavMesh(_navigationMesh);
			_navigationMesh = 0;
		}
		
		dtNavMesh *Mesh::GetDetourNavigationMesh()
		{
			return _navigationMesh;
		}
		
		void Mesh::DumpToOBJ(const char *path)
		{
			FileIO io(path);
			duDumpPolyMeshDetailToObj(*_polyMeshDetail, &io);
		}
	}
}

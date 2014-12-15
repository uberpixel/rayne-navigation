//
//  RNNPath.cpp
//  rayne-navigation
//
//  Created by Nils Daumann on 14.12.14.
//  Copyright (c) 2014 Uberpixel. All rights reserved.
//

#include "RNNPath.h"

namespace RN
{
	namespace navigation
	{
		Path::Path(Mesh *navMesh) :
		tolerance(RN::Vector3(4.0f)), _navMesh(navMesh)
		{
			_navMesh->Retain();
			_query = dtAllocNavMeshQuery();
			_query->init(_navMesh->GetDetourNavigationMesh(), 2048);
			
			_filter = new dtQueryFilter();
		}
		
		Path::~Path()
		{
			_navMesh->Release();
			dtFreeNavMeshQuery(_query);
			delete _filter;
		}
		
		bool Path::FindPath(const RN::Vector3& start, const RN::Vector3& target)
		{
			dtPolyRef startPoly;
			dtPolyRef targetPoly;
			
			_query->findNearestPoly(&start.x, &tolerance.x, _filter, &startPoly, nullptr);
			_query->findNearestPoly(&target.x, &tolerance.x, _filter, &targetPoly, nullptr);
			
			if(startPoly && targetPoly)
			{
				int outCount;
				std::vector<dtPolyRef> path(2048);
				
				_query->findPath(startPoly, targetPoly, &start.x, &target.x, _filter, path.data(), &outCount, static_cast<int>(path.capacity()));
				
				path.resize(outCount);
				
				if(outCount)
				{
					_path.resize(2048);
					
					_query->findStraightPath(&start.x, &target.x, path.data(), static_cast<int>(path.size()), reinterpret_cast<float *>(_path.data()), nullptr, nullptr, &outCount, static_cast<int>(_path.capacity()));
					_path.resize(outCount);
					
					std::reverse(_path.begin(), _path.end());
					return true;
				}
			}
			
			return false;
		}
		
		const RN::Vector3& Path::GetClosestPoint() const
		{
			return _path.at(_path.size() - 1);
		}
		
		void Path::PopPoint()
		{
			_path.pop_back();
		}
		
		bool Path::IsAtEnd()
		{
			return _path.empty();
		}
	}
}

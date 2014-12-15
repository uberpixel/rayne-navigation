//
//  RNNPath.h
//  rayne-navigation
//
//  Created by Nils Daumann on 14.12.14.
//  Copyright (c) 2014 Uberpixel. All rights reserved.
//

#ifndef __rayne_navigation__RNNPath__
#define __rayne_navigation__RNNPath__

#include <Rayne/Rayne.h>

#include "DetourNavMeshQuery.h"
#include "RNNMesh.h"

namespace RN
{
	namespace navigation
	{
		class Path
		{
		public:
			Path(Mesh *navMesh);
			~Path();
			
			bool FindPath(const RN::Vector3& start, const RN::Vector3& target);
			
			const RN::Vector3& GetClosestPoint() const;
			void PopPoint();
			bool IsAtEnd();
			
			RN::Vector3 tolerance;
			
		private:
			Mesh *_navMesh;
			
			dtPolyRef _startRef;
			dtPolyRef _targetRef;
			
			dtQueryFilter *_filter;
			dtNavMeshQuery *_query;
			
			std::vector<RN::Vector3> _path;
		};
	}
}

#endif /* defined(__rayne_navigation__RNNPath__) */

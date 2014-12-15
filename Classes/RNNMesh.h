//
//  RNNMesh.h
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

#ifndef __rayne_navigation__RNNMesh__
#define __rayne_navigation__RNNMesh__

#include <Rayne/Rayne.h>

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "RecastDump.h"

namespace RN
{
	namespace navigation
	{
		/// Recast build context.
		class BuildContext : public rcContext
		{
		public:
			BuildContext();
			
		protected:
			virtual void doLog(const rcLogCategory category, const char* msg, const int len);
		};
		
		struct FileIO : public duFileIO
		{
			FileIO(const char *path)
			{
				_file = fopen(path, "wb");
			}
	
			virtual ~FileIO()
			{
				fclose(_file);
			}
		
			virtual bool isWriting() const { return true; }
			virtual bool isReading() const { return false; }
			virtual bool write(const void* ptr, const size_t size) { fwrite(ptr, size, 1, _file); return true; }
			virtual bool read(void* ptr, const size_t size) { return false; }
			
			FILE *_file;
		};
		
		class Mesh : public RN::Object
		{
		public:
			enum PartitionType
			{
				Watershed,
				Monotone,
				Layers,
			};
			
			Mesh();
			Mesh(RN::Model *model);
			~Mesh();
			
			bool GenerateFromModel(RN::Model *model);
			bool GenerateFromModels(RN::Array *models);
			
			dtNavMesh *GetDetourNavigationMesh();
			
			void DumpToOBJ(const char *path);
			
			float _cellSize;
			float _cellHeight;
			float _agentHeight;
			float _agentRadius;
			float _agentMaxClimb;
			float _agentMaxSlope;
			float _regionMinSize;
			float _regionMergeSize;
			float _edgeMaxLen;
			float _edgeMaxError;
			float _vertsPerPoly;
			float _detailSampleDist;
			float _detailSampleMaxError;
			
		private:
			void Initialize();
			void Cleanup();
			
			PartitionType _partitionType;
			
			rcPolyMesh* _polyMesh;
			rcConfig _recastConfig;
			rcPolyMeshDetail* _polyMeshDetail;
			
			dtNavMesh* _navigationMesh;
			
			/*status = _navigationQuery->init(_navigationMesh, 2048);
			dtNavMeshQuery *_navigationQuery;*/
			
			RNDeclareMeta(Mesh)
		};
	}
}

#endif /* defined(__rayne_navigation__RNNMesh__) */

#pragma once

/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <glMeshData.h>

namespace DFHM {

	class GlMultiKeyMeshData {
	public:
		void setFaceTessellations(const std::vector<OGL::IndicesPtr>& src);
		void setEdgeTessellations(const std::vector<OGL::IndicesPtr>& src);

	protected:
		std::map<VertexPointAndNormal, size_t> _triVertexToIndexMap;
		std::map<VertexPoint, size_t> _edgeVertexToIndexMap;
		std::map<GLEdge, size_t> _edgeMap;
		std::vector<float> _triPoints, _triNormals, _edgePoints; // Appears to not being cleared
		std::vector<std::vector<unsigned int>> _triIndices, _edgeIndices;
		std::vector<OGL::IndicesPtr> _faceTessellations, _edgeTessellations;

	};

	inline void GlMultiKeyMeshData::setFaceTessellations(const std::vector<OGL::IndicesPtr>& src)
	{
		_faceTessellations = src;
	}

	inline void GlMultiKeyMeshData::setEdgeTessellations(const std::vector<OGL::IndicesPtr>& src)
	{
		_edgeTessellations = src;
	}

}
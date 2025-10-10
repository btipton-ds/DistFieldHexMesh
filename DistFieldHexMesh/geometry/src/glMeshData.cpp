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

#include <glMeshData.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

void GlMeshFaces::addTriangle(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2)
{
    Vector3d v0 = pt0 - pt1;
    Vector3d v1 = pt2 - pt1;
    Vector3d n = v0.cross(v1).normalized();

    for (int i = 0; i < 3; i++) {
        _glTriPoints.push_back((float)pt0[i]);
        _glTriNormals.push_back((float)n[i]);
    }

    for (int i = 0; i < 3; i++) {
        _glTriPoints.push_back((float)pt1[i]);
        _glTriNormals.push_back((float)n[i]);
    }

    for (int i = 0; i < 3; i++) {
        _glTriPoints.push_back((float)pt2[i]);
        _glTriNormals.push_back((float)n[i]);
    }

}

void GlMeshFaces::addFace(const Polygon& face)
{
    auto triFunc = [this](const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2) {
        Vector3d v0 = pt0 - pt1;
        Vector3d v1 = pt2 - pt1;
        Vector3d n = v0.cross(v1).normalized();

        for (int i = 0; i < 3; i++) {
            _glTriPoints.push_back((float)pt0[i]);
            _glTriNormals.push_back((float)n[i]);
        }

        for (int i = 0; i < 3; i++) {
            _glTriPoints.push_back((float)pt1[i]);
            _glTriNormals.push_back((float)n[i]);
        }

        for (int i = 0; i < 3; i++) {
            _glTriPoints.push_back((float)pt2[i]);
            _glTriNormals.push_back((float)n[i]);
        }
        };

    auto edgeFunc = [this](const Vector3d& pt0, const Vector3d& pt1) {
        for (int i = 0; i < 3; i++)
            _glEdgePoints.push_back((float)pt0[i]);

        for (int i = 0; i < 3; i++)
            _glEdgePoints.push_back((float)pt1[i]);
        };

    face.getTriPoints(triFunc, edgeFunc);

}

size_t GlMeshFaces::numTriVertices() const
{
    return _glTriPoints.size() / 3;
}

size_t GlMeshFaces::numEdgeVertices() const
{
    return _glEdgePoints.size() / 2;
}

VertexPoint::VertexPoint(const Vector3f& pt)
{
    for (int i = 0; i < 3; i++) {
        _iPoint[i] = (int)(pt[i] * 100000);
    }
}

bool VertexPoint::operator < (const VertexPoint& rhs) const
{
    return _iPoint < rhs._iPoint;
}

VertexPointAndNormal::VertexPointAndNormal(const Vector3f& pt, const Vector3f& normal)
{
    for (int i = 0; i < 3; i++) {
        _iPoint[i] = (int)(pt[i] * 100000);
        _iNormal[i] = (int)(normal[i] * 100000);
    }
}

bool VertexPointAndNormal::operator < (const VertexPointAndNormal& rhs) const
{
    if (_iPoint < rhs._iPoint)
        return true;
    else if (rhs._iPoint < _iPoint)
        return false;

    return _iNormal < rhs._iNormal;
}

GLEdge::GLEdge(unsigned int idx0, unsigned int idx1)
    : _idx0(idx0 < idx1 ? idx0 : idx1)
    , _idx1(idx0 < idx1 ? idx1 : idx0)
{
}

bool GLEdge::operator < (const GLEdge& rhs) const
{
    if (_idx0 < rhs._idx0)
        return true;
    else if (_idx0 > rhs._idx0)
        return false;

    return _idx1 < rhs._idx1;
}

from geoalchemy2.functions import GenericFunction
from geoalchemy2.comparator import Comparator

class cos(GenericFunction):
    name = 'cos'
    type = None

class sin(GenericFunction):
    name = 'sin'
    type = None

class pi(GenericFunction):
    name = 'pi'
    type = None

class ST_GeomFromText(GenericFunction):
    name = 'ST_GeomFromText'
    type = None

class ST_GeometryN(GenericFunction):
    name = 'ST_GeometryN'
    type = None

class ST_PointN(GenericFunction):
    name = 'ST_PointN'
    type = None

class ST_Collect(GenericFunction):
    name = 'ST_Collect'
    type = None

class ST_CollectionExtract(GenericFunction):
    name = 'ST_CollectionExtract'
    type = None

class Box2D(GenericFunction):
    name = 'Box2D'
    type = None

class Box3D(GenericFunction):
    name = 'Box3D'
    type = None

class ST_XMax(GenericFunction):
    name = 'ST_XMax'
    type = None

class ST_XMin(GenericFunction):
    name = 'ST_XMin'
    type = None

class ST_YMax(GenericFunction):
    name = 'ST_YMax'
    type = None

class ST_YMin(GenericFunction):
    name = 'ST_YMin'
    type = None

class ST_ZMax(GenericFunction):
    name = 'ST_ZMax'
    type = None

class ST_ZMin(GenericFunction):
    name = 'ST_ZMin'
    type = None

class ST_X(GenericFunction):
    name = 'ST_X'
    type = None

class ST_Y(GenericFunction):
    name = 'ST_Y'
    type = None

class ST_Z(GenericFunction):
    name = 'ST_Z'
    type = None

class ST_ConcaveHull(GenericFunction):
    name = 'ST_ConcaveHull'
    type = None

class ST_Extent(GenericFunction):
    name = 'ST_Extent'
    type = None

class ST_3DExtent(GenericFunction):
    name = 'ST_3DExtent'
    type = None

class ST_Envelope(GenericFunction):
    name = 'ST_Envelope'
    type = None

class ST_3DDistance(GenericFunction):
    name = 'ST_3DDistance'
    type = None

class ST_3DMaxDistance(GenericFunction):
    name = 'ST_3DMaxDistance'
    type = None

class ST_3DShortestLine(GenericFunction):
    name = 'ST_3DShortestLine'
    type = None

class ST_3DLongestLine(GenericFunction):
    name = 'ST_3DLongestLine'
    type = None

class ST_3DDWithin(GenericFunction):
    name = 'ST_3DDWithin'
    type = None

class ST_3DDFullyWithin(GenericFunction):
    name = 'ST_3DDFullyWithin'
    type = None

class ST_3DClosestPoint(GenericFunction):
    name = 'ST_3DClosestPoint'
    type = None

class ST_Touches(GenericFunction):
    name = 'ST_Touches'
    type = None

class ST_Within(GenericFunction):
    name = 'ST_Within'
    type = None

class ST_Overlaps(GenericFunction):
    name = 'ST_Overlaps'
    type = None

class ST_Contains(GenericFunction):
    name = 'ST_Contains'
    type = None

class ST_Crosses(GenericFunction):
    name = 'ST_Crosses'
    type = None

class ST_Intersects(GenericFunction):
    name = 'ST_Intersects'
    type = None

class ST_Disjoint(GenericFunction):
    name = 'ST_Disjoint'
    type = None

class ST_DWithin(GenericFunction):
    name = 'ST_DWithin'
    type = None

class ST_DFullyWithin(GenericFunction):
    name = 'ST_DFullyWithin'
    type = None

class ST_StraightSkeleton(GenericFunction):
    name = 'ST_StraightSkeleton'
    type = None

class ST_Distance(GenericFunction):
    name = 'ST_Distance'
    type = None

class ST_3DArea(GenericFunction):
    name = 'ST_3DArea'
    type = None

class ST_3DIntersection(GenericFunction):
    name = 'ST_3DIntersection'
    type = None

class ST_3DIntersects(GenericFunction):
    name = 'ST_3DIntersects'
    type = None

class ST_Affine(GenericFunction):
    name = 'ST_Affine'
    type = None

class ST_ConvexHull(GenericFunction):
    name = 'ST_ConvexHull'
    type = None

class ST_IsPlanar(GenericFunction):
    name = 'ST_IsPlanar'
    type = None

class ST_Buffer(GenericFunction):
    name = 'ST_Buffer'
    type = None

#### SFCGAL IMPORT

class SFCGAL_IsValid(GenericFunction):
    name = 'SFCGAL_IsValid'
    type = None

class SFCGAL_Volume(GenericFunction):
    name = 'SFCGAL_Volume'
    type = None

class SFCGAL_Area(GenericFunction):
    name = 'SFCGAL_Area'
    type = None

class SFCGAL_Area3D(GenericFunction):
    name = 'SFCGAL_Area3D'
    type = None

class SFCGAL_Distance(GenericFunction):
    name = 'SFCGAL_Distance'
    type = None

class SFCGAL_Distance3D(GenericFunction):
    name = 'SFCGAL_Distance3D'
    type = None

class SFCGAL_Intersects(GenericFunction):
    name = 'SFCGAL_Intersects'
    type = None

class SFCGAL_Intersects3D(GenericFunction):
    name = 'SFCGAL_Intersects3D'
    type = None

class SFCGAL_Contains3D(GenericFunction):
    name = 'SFCGAL_Contains3D'
    type = None

class SFCGAL_Intersection(GenericFunction):
    name = 'SFCGAL_Intersection'
    type = None

class SFCGAL_Intersection3D(GenericFunction):
    name = 'SFCGAL_Intersection3D'
    type = None

class SFCGAL_Difference(GenericFunction):
    name = 'SFCGAL_Difference'
    type = None

class SFCGAL_Difference3D(GenericFunction):
    name = 'SFCGAL_Difference3D'
    type = None

class SFCGAL_Union(GenericFunction):
    name = 'SFCGAL_Union'
    type = None

class SFCGAL_Union3D(GenericFunction):
    name = 'SFCGAL_Union3D'
    type = None

class SFCGAL_Convexhull(GenericFunction):
    name = 'SFCGAL_Convexhull'
    type = None

class SFCGAL_Convexhull3D(GenericFunction):
    name = 'SFCGAL_Convexhull3D'
    type = None

class SFCGAL_Extrude(GenericFunction):
    name = 'SFCGAL_Extrude'
    type = None

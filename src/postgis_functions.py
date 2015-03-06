from geoalchemy2.functions import GenericFunction

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

class ST_CollectionExtract(GenericFunction):
    name = 'ST_CollectionExtract'
    type = None

class ST_3DDistance(GenericFunction):
    name = 'ST_3DDistance'
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

#### SFCGAL IMPORT

class SFCGAL_Volume(GenericFunction):
    name = 'SFCGAL_Volume'
    type = None

class SFCGAL_Area(GenericFunction):
    name = 'SFCGAL_Area'
    type = None

class SFCGAL_3DArea(GenericFunction):
    name = 'SFCGAL_3DArea'
    type = None

class SFCGAL_Distance(GenericFunction):
    name = 'SFCGAL_Distance'
    type = None

class SFCGAL_3DDistance(GenericFunction):
    name = 'SFCGAL_3DDistance'
    type = None

class SFCGAL_Intersects(GenericFunction):
    name = 'SFCGAL_Intersects'
    type = None

class SFCGAL_3DIntersects(GenericFunction):
    name = 'SFCGAL_3DIntersects'
    type = None

class SFCGAL_Intersection(GenericFunction):
    name = 'SFCGAL_Intersection'
    type = None

class SFCGAL_3DIntersection(GenericFunction):
    name = 'SFCGAL_3DIntersection'
    type = None

class SFCGAL_Difference(GenericFunction):
    name = 'SFCGAL_Difference'
    type = None

class SFCGAL_3DDifference(GenericFunction):
    name = 'SFCGAL_3DDifference'
    type = None

class SFCGAL_Union(GenericFunction):
    name = 'SFCGAL_Union'
    type = None

class SFCGAL_3DUnion(GenericFunction):
    name = 'SFCGAL_3DUnion'
    type = None

class SFCGAL_Convexhull(GenericFunction):
    name = 'SFCGAL_Convexhull'
    type = None

class SFCGAL_3DConvexhull(GenericFunction):
    name = 'SFCGAL_3DConvexhull'
    type = None

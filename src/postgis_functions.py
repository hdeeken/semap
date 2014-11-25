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

class ST_3DDistance(GenericFunction):
    name = 'ST_3DDistance'
    type = None

class ST_3DIntersection(GenericFunction):
    name = 'ST_3DIntersection'
    type = None

class ST_Affine(GenericFunction):
    name = 'ST_Affine'
    type = None

from lxml.builder import E
from lxml import etree
import xml.etree.ElementTree as ET

#xml、事前条件、効果、アクション名、refineが必要な記号？
class ActionTemp:
    __slots__ = ['precond', 'action', 'effects', 'xml', 'cos']

    def __init__(self,xml, p, e, a, cos):
        self.precond = p
        self.action =  a
        self.xml = xml
        self.effects =  e
        self.cos = cos

    def to_string(self, property):
        s =""
        for p in property :
            
            s += str(p) 
        return s


class Obstacle:
    __slots__ = ['name', 'position']
    
    def __init__(self, name, position):
        self.name = name
        self.position = position

class Location:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Target:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Belief:
    __slots__ = ['distribution']
    
    def __init__(self, distribution):
        self.distribution = distribution

    def update(self, belief):
        return belief


class Constraint:
    __slots__ = ['type', 'xml', 'params']
    
    def __init__(self, type, xml, params):
        self.type = type
        self.xml = xml        
        self.params = params
    def __copy__(self):
        cls = self.__class__
        newobject = cls.__new__(cls)
        newobject.__dict__.update(self.__dict__)
        newobject.params = self.params
        newobject.xml = self.xml
        newobject.type = self.type
        return newobject

class ConstraintOperativeSubspace:
    def __init__(self, variables, constraints):
        self.variables = variables
        self.constraints = constraints
# Christopher Sandoval 13660
# 29/03/2019
# SR6: Transformations

import software_renderer as sr
from collections import namedtuple

V2 = namedtuple('Vertex2',['x', 'y'])
V3 = namedtuple('Vertex3',['x', 'y', 'z'])

sr.glInit()
sr.glCreateWindow(800,800)
sr.glViewPort(0,0,800,800)
sr.glClearColor(0,0,0)


sr.glLookAt(V3(0,0.6,2.5),V3(0,0.2,0),V3(0.6,1,0))
sr.glLoadObjTexture('sphere.obj', '', '')
sr.glFinish()

    
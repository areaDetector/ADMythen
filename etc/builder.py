from iocbuilder import Device, AutoSubstitution
from iocbuilder.arginfo import *

from iocbuilder.modules.asyn import AsynPort
from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, includesTemplates, makeTemplateInstance

@includesTemplates(ADBaseTemplate)
class _mythenTemplate(AutoSubstitution):
    TemplateFile="mythen.template"

class mythenDetector(AsynPort):
    ''' Dectris Mythen strip detector device support. '''
    Dependencies = (ADCore,)
    _SpecificTemplate = _mythenTemplate
    UniqueName = "PORT"
    def __init__(self, PORT, IPPORT, BUFFERS=50, MEMORY=-1, **args):
        # Init the parent
        self.__super.__init__(PORT)
        # Update the attributes of self from the command line args
        self.__dict__.update(locals())
        # Make an instance of the template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)

    # __init__ arguments
    ArgInfo = _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
                PORT = Simple('Port name for the detector', str),
                IPPORT = Ident('Asyn IP Port.', AsynPort),
                BUFFERS = Simple('Maximum number of NDArray buffers to be created for callbacks', int),
                MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer for driver and all plugins', int)
            )
    
    LibFileList = ['mythen']
    DbdFileList = ['mythenSupport']
    
    def Initialise(self):
        print '# mythenConfig(portName, IPPORT, maxBuffers, maxMemory)'
        print 'mythenConfig("%(PORT)s", "%(IPPORT)s", "%(BUFFERS)d", "%(MEMORY)d"' % self.__dict__


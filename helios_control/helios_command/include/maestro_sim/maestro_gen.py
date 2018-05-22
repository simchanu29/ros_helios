import pybindgen
import sys

mod = pybindgen.Module('Maestro')
mod.add_include('"maestro.h"')
mod.add_function('maestroGetPosition',
                 pybindgen.retval ('int'),
                 [pybindgen.param ('int', 'fd'),
                  pybindgen.param ('unsigned char', 'channel')])
mod.add_function('maestroSetTarget',
                 pybindgen.retval ('int'),
                 [pybindgen.param ('int', 'fd'),
                  pybindgen.param ('unsigned char', 'channel'),
                  pybindgen.param ('unsigned short', 'target')])
mod.add_function('maestroSetSpeed',
                 pybindgen.retval ('int'),
                 [pybindgen.param ('int', 'fd'),
                  pybindgen.param ('unsigned char', 'channel'),
                  pybindgen.param ('unsigned short', 'target')])
mod.add_function('maestroSetAccel',
                 pybindgen.retval ('int'),
                 [pybindgen.param ('int', 'fd'),
                  pybindgen.param ('unsigned char', 'channel'),
                  pybindgen.param ('unsigned short', 'target')])
mod.add_function('maestroConnect',
                 pybindgen.retval ('int'),
                 [pybindgen.param ('const char *', 'device')])
mod.generate(sys.stdout)
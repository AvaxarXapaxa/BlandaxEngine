from distutils.core import setup, Extension
from Cython.Build import cythonize
import numpy

setup(
    ext_modules = cythonize('blandax_engine.pyx', language='c++', compiler_directives={
            'cdivision': True,
            'language_level': 3,
            'wraparound': False,
        }, build_dir= 'build'),
    include_dirs = [numpy.get_include()]
)

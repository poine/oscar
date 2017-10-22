from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext

##
# workaround this bug:
# http://stackoverflow.com/questions/8106258/cc1plus-warning-command-line-option-wstrict-prototypes-is-valid-for-ada-c-o
import distutils, Cython
class my_build_ext(Cython.Distutils.build_ext):
    def build_extensions(self):
        distutils.sysconfig.customize_compiler(self.compiler)
        try:
            self.compiler.compiler_so.remove("-Wstrict-prototypes")
        except (AttributeError, ValueError):
            pass
        build_ext.build_extensions(self)
##
# extra_compile_args=["-std=c++11", "-Wno-cpp"] is for removing the deprecation warning
#  because defining NPY_NO_DEPRECATED_API to NPY_1_7_API_VERSION breaks compilation



ext = Extension(
    "pyrobcape",        # name of extension
    ["robcape.pyx"],      # filename of our Pyrex/Cython source
    language="c++",    # this causes Pyrex/Cython to create C++ source
    include_dirs=[],
    extra_compile_args=["-std=c++11", "-Wno-cpp"],
    libraries=["m", "rt", "pthread", "roboticscape"],
    library_dirs=[],
    runtime_library_dirs=[],
)

setup(
    name = 'pyrobcape',
    ext_modules = [ext],
    cmdclass = {'build_ext': my_build_ext}
)

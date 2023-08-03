from distutils.core import setup, Extension

setup(name='sonar_data_processor', version='1.0',  \
      ext_modules=[Extension(
            name = 'sonar_data_processor',
            sources = [
                  'sonar_data_processor.cpp',
                  ],
            include_dirs = [
                  '/usr/include',
                  './pybind11',
                  ],
            #define_macros = [('DEBUG_ENABLED', None)],
            extra_compile_args = ["-O3", "-Wall", "-shared", "-std=c++11", "-fPIC", "-pthread"],
            )]
      )
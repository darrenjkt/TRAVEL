from setuptools import find_packages, setup
from pybind11.setup_helpers import Pybind11Extension, build_ext
import os

pcl_libs = ["pcl_common-1.8", "pcl_filters-1.8"]

setup(name='travel',
      version='1.0',
      description='Ground estimation with TRAVEL',      
      author='Darren Tsai',
      author_email='d.tsai@acfr.usyd.edu.au',
      license='MIT',
      cmdclass={'build_ext': build_ext},
      ext_modules=[
          Pybind11Extension(
            name='travel', 
            sources=['travel_api.cpp',],
            include_dirs=[
            os.environ.get("EIGEN_INCLUDE_DIR", "/usr/include/eigen3/"),
            "/usr/include/pcl-1.8/"],          
            extra_compile_args=['-std=c++14'],
          )],
      
      python_requires=">=3.7",
)
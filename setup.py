from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['grasp_server_library']
d['scripts'] = []
d['package_dir'] = {'': 'python'}

setup(**d)

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bender_behaviors'],
    # obs: this should point to my_script_1.py ... (not folders)
    #scripts=['src/bender_behaviors'],
    package_dir={'': 'src'}
)

setup(**d)

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bender_gui_launch'],
    # obs: this should point to my_script_1.py ... (not folders)
    #scripts=['src/macros'],
    package_dir={'': 'src'}
)

setup(**d)

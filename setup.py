from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	scripts=['scripts/tank_robot_llc_drive.py']
)

setup(**d)

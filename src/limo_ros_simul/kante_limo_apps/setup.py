from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'kante_limo_apps',
        'kante_limo_apps.sensor_per',
        'kante_limo_apps.control',
        'kante_limo_apps.mission',
    ],
    package_dir={'': 'src'},
)

setup(**d)

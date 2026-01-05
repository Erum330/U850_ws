from setuptools import setup
import os
from glob import glob

package_name = 'uf850_ttt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The following two lines are what "install" your folders
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erum',
    maintainer_email='58556663+Erum330@users.noreply.github.com',
    description='Tic-Tac-Toe for UF850',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_ttt_pieces = uf850_ttt.spawn_ttt_pieces:main',
            'pick_place_one = uf850_ttt.pick_place_one:main',
            'cartesian_automation = uf850_ttt.cartesian_automation:main',
            # We will add your game_logic.py here in the next step
        ],
    },
)
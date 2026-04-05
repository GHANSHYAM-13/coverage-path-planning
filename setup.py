from glob import glob
import os

from setuptools import setup

package_name = 'my_coverage'


def package_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch/*')),
        (os.path.join('share', package_name, 'params'), package_files('params/*')),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            package_files('behavior_trees/*'),
        ),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jd',
    maintainer_email='jd@example.com',
    description='Reusable OpenNav coverage-server package for cleaning robots.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_executor = my_coverage.coverage_executor:main',
            'gui_coverage = my_coverage.gui_coverage:main',
            'polygon_drawer = my_coverage.polygon_drawer:main',
        ],
    },
)

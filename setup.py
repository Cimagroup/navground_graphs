from setuptools import setup, find_packages

package_name = 'navground_graphs'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Javier Perera-Lago',
    maintainer_email='jperera@us.es',
    description='This package describes a behavior modulation that models the environment state in terms of graphs to prevent and avoid collisions and deadlocks in narrow scenarios.',
    license='TODO: License declaration',
    tests_require=['pytest'],
)

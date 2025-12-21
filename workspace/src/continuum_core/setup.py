from setuptools import find_packages, setup

package_name = 'continuum_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'continuum',
    ],
    zip_safe=True,
    maintainer='Antonio Zugaldia',
    maintainer_email='antonio@zugaldia.com',
    description='Continuum core package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heartbeat_node = continuum_core.health.heartbeat_node:main',
            'echo_node = continuum_core.health.echo_node:main',
        ],
    },
)

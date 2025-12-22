from setuptools import find_packages, setup

package_name = "continuum_desktop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/launch_desktop.py"]),
    ],
    install_requires=["setuptools", "continuum"],
    zip_safe=True,
    maintainer="Antonio Zugaldia",
    maintainer_email="antonio@zugaldia.com",
    description="Continuum desktop package",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)

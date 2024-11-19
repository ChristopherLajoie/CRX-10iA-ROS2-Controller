from setuptools import setup, find_packages

package_name = 'moveit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Include your Python package
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = moveit_controller.controller:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'search_det'
submodule_name = 'search_det/SLAM'
#submodule_name2 = 'search_det/SLAM/skimage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coders',
    maintainer_email='coders@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'search_det = search_det.search_det:main'
        ],
    },
)

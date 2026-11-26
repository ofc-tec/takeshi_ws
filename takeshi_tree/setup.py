from setuptools import find_packages, setup

package_name = 'takeshi_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscar',
    maintainer_email='ofc1227@tec.mx',
    description='Behavior trees for Takeshi HSR',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = takeshi_tree.task_manager:main',
        ],
    },
)
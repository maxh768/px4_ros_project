from setuptools import find_packages, setup

package_name = 'gnc_offboard'

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
    maintainer='maxho',
    maintainer_email='mhowel30@vols.utk.edu',
    description='Package for custom control modules',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'local_position_echo = gnc_offboard.local_position_echo:main',
    ],
    },
)

from setuptools import find_packages, setup

package_name = 'bin_detector'

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
    maintainer='mscrobotics2425laptop5',
    maintainer_email='wang.yiting@student.zy.cdut.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'bin_dectection_node = bin_detector.bin_detect:main',
        'test_dectection_node = bin_detector.test_bin:main',
        
        ],
    },
)

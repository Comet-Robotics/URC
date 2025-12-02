from setuptools import find_packages, setup

package_name = 'spectrometer_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',  ['config/caldata.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ana',
    maintainer_email='ana.wise06@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spectrometer_node = spectrometer_publisher.SpectrometerNode:main',
        ],
    },
)

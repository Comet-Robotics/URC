from setuptools import find_packages, setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/img', ['img/Human_faces.jpg']),
        ('share/' + package_name + '/models', ['models/haarcascade_frontalface_default.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danwalkr79',
    maintainer_email='danwalkr79@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher = tracking.image_publisher:main',
            'image_display = tracking.image_subscriber:main',
            'still_image = tracking.still_publisher:main',
            'annotations = tracking.test_annotations:main',
            'face_track_haar = tracking.face_track_haar:main',
            'annotation_node = tracking.tracking_annotation:main'
        ],
    },
)

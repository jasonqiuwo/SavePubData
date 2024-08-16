from setuptools import setup

package_name = 'my_transforms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jason',
    maintainer_email='23819499@sun.ac.za',
    description='Transform publishers for rotating lidar and mapping',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder = my_transforms.encoder:main',
            'fusion_filtered = my_transforms.fusion_filtered:main',
            'laser_transform_publisher = my_transforms.laser_transform_publisher:main',
            'lidar_encoder_fusion = my_transforms.lidar_encoder_fusion:main',
            'read_h5 = my_transforms.read_h5:main',
            'rgbd_print = my_transforms.rgbd_print:main',
            'rgbd_save = my_transforms.rgbd_save:main',
            'scan_print = my_transforms.scan_print:main',
            'scan_save = my_transforms.scan_save:main',
        ],
    },
)


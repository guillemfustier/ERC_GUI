from setuptools import find_packages, setup

package_name = 'unity'

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
    maintainer='guillem',
    maintainer_email='al426661@uji.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ejemplo = unity.moverse_esperar:main',
            'image_subscriber = unity.image_subscriber:main',
            'cam_pub = unity.cam_pub:main'
        ],
    },
)

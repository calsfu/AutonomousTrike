from setuptools import find_packages, setup

package_name = 'perception_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'segment-anything-fast', 
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='coler',
    maintainer_email='colerezi30@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fast_sam = perception_py.fast_sam:main', 
        ],
    },
)

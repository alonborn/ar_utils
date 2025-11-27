from setuptools import find_packages, setup

package_name = 'ar_utils'

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
    maintainer='alon',
    maintainer_email='alonborn@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
    'test': ['pytest'],
    },  
    entry_points={
        'console_scripts': [
            'move_ar = ar_utils.move_ar:main',  # Entry point for your node
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'signal_measure'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/signal_measure_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nilsj',
    maintainer_email='nilsjor@kth.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_pos = signal_measure.get_pos:main',
            'get_signal = signal_measure.get_signal:main',
            'visualize_signal = signal_measure.visualize_signal:main'
        ],
    },  
)

from setuptools import setup

package_name = 'jrb_hardware_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Axel Mousset',
    maintainer_email='axel@mousset.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sio_bridge = jrb_hardware_bridge.hardware_bridge_sio:main',
            'can_bridge = jrb_hardware_bridge.hardware_bridge_can:main',
            'uart_bridge = jrb_hardware_bridge.hardware_bridge_uart:main',
        ],
    },
)

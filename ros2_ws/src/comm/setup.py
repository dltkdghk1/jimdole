from setuptools import setup

package_name = 'comm'

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
    maintainer='SSAFY',
    maintainer_email='trop_ical@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt = comm.mqtt:main',
            'gate_receive = comm.gate_receive:main',
            'photo_send = comm.photo_send:main',
            'status_receive = comm.status_receive:main',
            'status_send = comm.status_send:main',
        ],
    },
)

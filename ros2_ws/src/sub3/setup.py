from setuptools import setup

package_name = 'sub3'

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
    maintainer='user',
    maintainer_email='mgko@morai.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aws_client = sub3.aws_client:main',            
            'tf_detector = sub3.tf_detector:main',                                  
            'iot_udp = sub3.iot_udp:main'                
        ],
    },
)

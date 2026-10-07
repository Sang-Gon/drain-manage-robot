from setuptools import find_packages, setup

package_name = 'drain_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/comms_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='ReportDrain -> server_bridge 토픽 어댑터',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'comms_adapter_node = drain_comms.comms_adapter_node:main',
        ],
    },
)

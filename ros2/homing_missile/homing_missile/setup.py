from setuptools import setup

package_name = 'homing_missile'

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
    maintainer='alek',
    maintainer_email='alek@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher_node = homing_missile.camera_publisher_node:main',
            'web_ui_node = homing_missile.web_ui_node:main',
        ],
    },
)

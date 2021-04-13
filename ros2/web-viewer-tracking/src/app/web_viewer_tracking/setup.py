from setuptools import setup

package_name = 'web_viewer_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bit-wrangler',
    maintainer_email='bit-wrangler@nunya.business',
    description='Package for serving a simple tracking UI via web browser',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_viewer_node = web_viewer_tracking.web_viewer_node:main'
        ],
    },
)

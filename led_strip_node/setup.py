from setuptools import setup

package_name = 'led_strip_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #py_modules=[
    #    'led_strip'
    #],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Stollenwerk',
    maintainer_email='cstollen@cit-ec.uni-bielefeld.de',
    description='LED strip controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_strip_listener = led_strip_node.led_strip:main',
        ],
    },
)

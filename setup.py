from setuptools import setup

package_name = 'caster_node'

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
    maintainer='Fredrik Löfgren',
    maintainer_email='fredrik@dynorobotics.se',
    description='Package to control holonomic drive platform with caster wheels',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'caster = caster_node.node:main',
        ],
    },
)

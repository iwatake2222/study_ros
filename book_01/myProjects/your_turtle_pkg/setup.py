from setuptools import setup

package_name = 'your_turtle_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move=your_turtle_pkg.moveTurtle:main',
            'spawn=your_turtle_pkg.spawnTurtle:main',
            'bg_param=your_turtle_pkg.bg_paramTurtle:main',
            'rotate=your_turtle_pkg.rotateTurtle:main',
            'mvTB3=your_turtle_pkg.mvTB3:main',
        ],
    },
)

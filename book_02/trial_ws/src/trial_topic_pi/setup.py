from setuptools import setup

package_name = 'trial_topic_pi'

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
    maintainer='iwatake',
    maintainer_email='iwatake@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publisher = trial_topic_pi.trial_publisher_pi:main",
            "subscriber = trial_topic_pi.trial_subscriber_pi:main"
        ],
    },
)

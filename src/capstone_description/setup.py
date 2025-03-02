from setuptools import setup

package_name = 'capstone_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/package_name']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of your package',
    entry_points={
        'console_scripts': [
            'motor_control_ros2 = capstone_description.motor_control_ros2:main',  # Ensure this points to your script's main function
        ],
    },
)

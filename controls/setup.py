from setuptools import find_packages, setup

package_name = 'controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redpi',
    maintainer_email='scott.blankenship00@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "esc_driver = controls.esc_driver:main",
            "balloon_detect_control = controls.balloon_pi:main",
            "mode_switch = controls.mode_switcher:main"
        ],
    },
)

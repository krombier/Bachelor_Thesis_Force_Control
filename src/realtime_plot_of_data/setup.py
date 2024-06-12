from setuptools import find_packages, setup

package_name = 'realtime_plot_of_data'

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
    maintainer='simi',
    maintainer_email='simon-a.burri@tbwnet.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'force_torque_plotter = realtime_plot_of_data.force_torque_plotter:main',            #sollte hier kein Abstand hin?????zwischen package name und program oder macht das der Punkt?
        ],
    },
)

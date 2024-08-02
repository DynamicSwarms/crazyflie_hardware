from setuptools import find_packages, setup

package_name = 'crtp_driver'
data_files = []
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyradio.launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyradio_sim.launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyradio_pure.launch.py']))




setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='winni',
    maintainer_email='winni@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyradio = crtp_driver.crazyradio_node:main',
            'crazyradio_sim = crtp_driver.crazyradio_node_sim:main',
            'radiolistener = crtp_driver.radio_listener:main',
            "crazyflie =  crtp_driver.crazyflie_node:main",
            "crtp_broadcaster =  crtp_driver.crtp_broadcaster:main"
        ],
    },
)

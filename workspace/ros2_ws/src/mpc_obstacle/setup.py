from setuptools import find_packages, setup

package_name = 'mpc_obstacle'

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
    maintainer='oli',
    maintainer_email='oliver.fellenz@stud.tu-darmstadt.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_CL_obstacle = mpc_obstacle.mpc_CL_obstacle:main',
            'adaptiveMPC_CL = mpc_obstacle.adaptiveMPC_CL:main',
            'slackMPC_CL = mpc_obstacle.slackMPC_CL:main',
            'mpc_CL_Nc = mpc_obstacle.mpc_CL_Nc:main ' ,
            'mpc_CL_dynObs = mpc_obstacle.mpc_CL_dynObs:main'       ],
    },
)

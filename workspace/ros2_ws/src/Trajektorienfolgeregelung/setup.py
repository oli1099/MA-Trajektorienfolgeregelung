from setuptools import find_packages, setup

package_name = 'MPC_Trajektorienfolgeregelung'

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
            
            'trajectoryPControllerTime = Trajektorienfolgeregelung.trajectoryPControllerTime:main',
            'mpc_PD_controller = Trajektorienfolgeregelung.mpc_PD_controller:main',
            'Trajectorytracking = Trajektorienfolgeregelung.Trajectorytracking:main',
            'PurePursuit = Trajektorienfolgeregelung.PurePursuit:main'
        ],
    },
)

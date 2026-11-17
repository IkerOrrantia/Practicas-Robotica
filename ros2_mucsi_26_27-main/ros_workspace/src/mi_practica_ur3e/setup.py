from setuptools import find_packages, setup

package_name = 'mi_practica_ur3e'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laboratorio',
    maintainer_email='estudiante@deusto.es',
    description='Practica 1 de Robotica y Automatizacion Inteligente',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_evaluador = mi_practica_ur3e.nodo_evaluador:main',
        ],
    },
)

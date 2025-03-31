from setuptools import setup

package_name = 'bag_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Deve corrispondere alla cartella che contiene camera_detection.py
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TuoNome',
    maintainer_email='tuo@email.com',
    description='Nodo per la rilevazione della bag con YOLO',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_detection = bag_detection.camera_detection:main'
        ],
    },
)


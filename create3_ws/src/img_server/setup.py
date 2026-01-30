from setuptools import setup
import os
from glob import glob

package_name = 'img_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # TEMPLATES
        (os.path.join('lib', package_name, 'templates'),
            glob('img_server/templates/*.html')),

        # STATIC FILES (solo archivos, NO directorios)
        (os.path.join('lib', package_name, 'static'),
            glob('img_server/static/*.css')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jlmre',
    maintainer_email='jlmre@todo.com',
    description='Image server with Flask',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'img_server_node = img_server.img_server_node:main',
        ],
    },
)


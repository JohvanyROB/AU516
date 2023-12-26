from setuptools import setup

package_name = 'au516_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='The navigation strategy will be implemented in this package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "agent.py = au516_nav.agent:main"
        ],
    },
)

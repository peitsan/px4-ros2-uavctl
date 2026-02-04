from setuptools import setup, find_packages

setup(
    name='vins_offboard_hover',
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools'],
    author='PX4 Dev Team',
    description='VINS-Fusion based offboard hover controller',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'vins_offboard_hover = px4_hexctl.vins_offboard_hover:main',
        ],
    },
)

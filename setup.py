from setuptools import setup

setup(
    name="faith-ros",
    version='0.0.1',
    packages=[],
    py_modules=['inference'],
    package_dir={'': 'src/py'},
    install_requires=['setuptools'],
    zip_safe=True,
    author='Degik',
    author_email='prodax9900@gmail.com',
    description='Faith-ROS inference node setup',
    license='MIT license',
    entry_points={
        'console_scripts': [
            'inference = inference:main',
        ],
    },
)
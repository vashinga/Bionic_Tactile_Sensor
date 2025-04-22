from setuptools import setup, find_packages

setup(
    name='bts_gelsight',
    version='0.1.0',
    description='Installing required packages for bts_gelsight.',
    author='Hamid Manouchehri',
    author_email='hmanouch@buffalo.edu',
    url='https://github.com/Hamid-Manouchehri/bts_gelsight.git',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'pandas',
        'matplotlib',
        'PyYAML',
        'opencv-python',
        'scipy',
        'ur_rtde',
        'pylsl',
        'dynamixel-sdk',
        'pyxdf',
        'pybind11',
        'phidget22',
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',  # Or your chosen license
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',         # Specify the Python versions you support
)

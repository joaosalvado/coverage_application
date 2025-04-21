from setuptools import setup, find_packages

setup(
    name='application',
    version='0.0.0',
    packages=find_packages(where='python-src'),
    package_dir={'': 'python-src'},
    install_requires=[
        'py_trees',
        'rospy',
        'geometry_msgs',
        'actionlib',
    ],
    zip_safe=False,
)

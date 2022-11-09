import os
from setuptools import setup

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "adcs",
    version = "0.0.1",
    author = "Siddharth Deore",
    author_email = "siddharthdeore@gmail.com",
    description = ("ADCS Toolbox with spacecraft systems such as"
                   "VSCMG, ReactionWheels "),
    license = "BSD",
    keywords = "adcs vscmg",
    url = "http://deore.in/",
    packages=['adcs'],
    package_dir={'adcs': 'adcs'},
    package_data={'adcs': ['*.so','*.pyd']},
    include_package_data = True,   
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Topic :: Utilities",
        "License :: OSI Approved :: BSD License",
    ],
)

from setuptools import setup, find_packages

setup(
    name="daxie",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "grpcio",
        "numpy==2.3.0",
        "protobuf==6.31.1",
        "scipy==1.15.3",
        "setuptools==80.9.0",
        "torch==2.7.1",
        "viser==0.2.23",
        "yourdfpy==0.0.57",
        "pyroki @ git+https://github.com/chungmin99/pyroki.git@f234516",
    ],
    author="Ilya Zisman",
    description="Let's you teleop your lerobot with a mobile phone",
    url="https://github.com/suessmann/daxie",
)

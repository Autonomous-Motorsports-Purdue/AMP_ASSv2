from setuptools import setup, find_packages

setup(
    name="AMP-CLI",
    version="0.2",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "click",
        "docker",
        "wheel",
    ],
    entry_points={
        "console_scripts": [
            "amp-cli = amp_cli:cli",
        ],
    },
)

from setuptools import setup

setup(
    name="AMP-CLI",
    version="0.2",
    py_modules=["amp_cli"],
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

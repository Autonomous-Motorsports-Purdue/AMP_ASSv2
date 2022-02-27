from setuptools import setup

setup(
    name='AMP-CLI',
    version='0.1',
    py_modules=['amp_cli'],
    install_requires=[
        'click',
        'docker',
        'os',
        'subprocess',
    ],
    entry_points={
        'console_scripts': [
            'amp-cli = amp_cli:cli',
        ],
    },
)

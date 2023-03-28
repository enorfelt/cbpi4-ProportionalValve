from setuptools import setup

from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(name='cbpi4-ProportionalValve',
  version='0.0.2',
  description='CraftBeerPi4 Plugin to control a motorized proportional valve using the MCP4725 board.',
  author='Erik Norfelt',
  author_email='enorfelt@outlook.com',
  url='https://github.com/enorfelt/cbpi4-ProportionalValve',
  include_package_data=True,
  package_data={
    # If any package contains *.txt or *.rst files, include them:
  '': ['*.txt', '*.rst', '*.yaml'],
  'cbpi4-ProportionalValve': ['*','*.txt', '*.rst', '*.yaml']},
  packages=['cbpi4-ProportionalValve'],
  install_requires=[
    'cbpi4>=4.1.6',,
    'Adafruit-Blinka',
    'adafruit-circuitpython-mcp4725',
  ],
  long_description=long_description,
  long_description_content_type='text/markdown'
)
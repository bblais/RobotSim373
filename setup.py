import RobotSim373

from distutils.core import setup
from distutils.extension import Extension

setup(
  name = 'RobotSim373',
  version=RobotSim373.__version__,
  description="RobotSim373",
  author="Brian Blais",
  packages=['RobotSim373'],
  install_requires=[
          'box2d',
          'python-pptx',
          'matplotlib',
      ],
)



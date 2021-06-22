# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

## [0.1.3] - 2021-06022
### Changed
- Quaternion conversion now conforms to ROS/TF2 conventions
- Fix bug in geometry_msgs.frame_to_numpy that would lead to zero 
  position/translation
- Fix bug in geometry_msgs.numpy_to_frame that would trigger an error in NumPy
- Improve checking for/converting dtypes

## [0.1.2] - 2021-06-06
### Added
- `__version__` string in top-level `__init__.py`

### Changed
- Fix bug in geometry_msgs.numpy_to_frame that would yield a TypeError
- Removed erroneously registered geometry_msgs.numpy_to_frame handler for
  message types `PoseStamped`, `TransformStamped`


## [0.1.1] - 2021-06-05
### Added
- Basic API functionality (`to_numpy`, `to_message`, `converts_to_numpy`,
  `converts_to_message`)
- Conversion handlers for geometry_msgs
- Rudimentary unittests
- Preliminary documentation in README.md

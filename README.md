## Depth Covariance Factors in GTSAM

Depth covariance prior depth factors used in bundle adjustment for "Learning a Depth Covariance Function".

## Installation

First, GTSAM must be installed with python bindings.  We are using the GTSAM version 4.2a7.

Please note that this library and GTSAM must be consistent in building with the CXX flag "-march=native".  By default, we have commented out line 5 of CMakeLists.txt which enables this flag.  If you wish to build with this flag, use a command such as 
```
cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.8.15 -DCMAKE_CXX_FLAGS="-march=native"
```
and uncomment line 5 when building this project.

To install, follow these instructions (with desired python version of current environment and the same one that GTSAM python bindings were installed with):

```
mkdir build
cd build
cmake .. -DGTSAM_PYTHON_VERSION=3.8.15
make
make python-install
```

To test the installation, run tests, such as:
```
python python/tests/test_example.py
```
or go into the build directory and run
```
make check
```

## Citation
If you found this code/work to be useful in your own research, please consider citing the following:
```bibtex
@inproceedings{dexheimer2023depthcov,
  title={{Learning a Depth Covariance Function},
  author={Eric Dexheimer and Andrew J. Davison},
  booktitle={IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR).},
  year={2023},
  }
}
```

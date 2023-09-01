## gtsam_gpdepth

Depth covariance prior depth factors from "Learning a Depth Covariance Function".

## Installation

First, GTSAM must be installed with python bindings.

To install, follow these instructions (with desired python version of current environment and the same one that GTSAM python bindings were installed with):

```
mkdir build
cd build
cmake .. -DGTSAM_PYTHON_VERSION=3.6.13`
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
If you found this code/work to be useful in your own research, please considering citing the following:
```bibtex
@inproceedings{dexheimer2023depthcov,
  title={{Learning a Depth Covariance Function},
  author={Eric Dexheimer and Andrew J. Davison},
  booktitle={IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR).},
  year={2023},
  }
}
```
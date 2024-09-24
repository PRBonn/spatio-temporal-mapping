<div align="center">
  <h1>Spatio-Temporal Mapping</h1>
    <a href="https://github.com/PRBonn/spatio-temporal-mapping#Installation"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/PRBonn/spatio-temporal-mapping#Usage"><img src="https://img.shields.io/badge/python-3670A0?style=flat-square&logo=python&logoColor=ffdd54" /></a>
    <a href="https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lobefaro2024iros.pdf"><img src="https://img.shields.io/badge/Paper-pdf-<COLOR>.svg?style=flat-square" /></a>
    <a href="https://github.com/PRBonn/spatio-temporal-mapping/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg?style=flat-square" /></a>

<p>
  <img src="https://github.com/PRBonn/spatio-temporal-mapping/blob/main/images/first_image.png" width="400"/>
</p>

<p>
  <i>Spatio-Temporal Mapping is a RGB-D odometry and mapping system specifically thought for dynamic environments such as agriculture setting.</i>
</p>

</div>

## Dependencies and Support
In order to be able to install the python package you need the essential software installable with the following command on Ubuntu:

```
sudo apt-get install --no-install-recommends -y build-essential cmake pybind11-dev python3-dev python3-pip
```

Then, all the dependencies will be handled by the system, you just need to follow the instructions in the [next section](#installation).

If you want to have dependencies installed on your machine, you can run the following command on Ubuntu:

```
sudo apt-get install libeigen3-dev libopencv-dev libtbb-dev libceres-dev 
```

For [tsl-robin map](https://github.com/Tessil/robin-map) and [Sophus](https://github.com/strasdat/Sophus), please refer to the relative github repos for installation from source.

**NOTE**: this software has been tested only on Ubuntu 22.04 machines, we do not ensure support for other platforms right now.

Neverthless, if you are on Ubuntu 20.04, be sure to install [ceres-solver](http://ceres-solver.org/installation.html) version *2.2.0* from source and turn off the option USE_SYSTEM_TBB for CMake (this will avoid to use the old version of openTBB).

**NOTE 2**: if you decide to not use your system installed version of Ceres, then be ready to huge memory consumption during building. For this reason, if you don't have a big RAM, just install libceres-dev with the command above.

## Installation
All you have to do is to clone this repo:

```
git clone https://github.com/PRBonn/spatio-temporal-mapping
```

and install it with:

```
make install
```

## Usage
This software gives you the possibility to perform three different tasks:

- Mapping (generate a map from a sequence of RGBD images by computing the odometry or with given poses)
    ```
    st_mapping-mapping --help
    ```
- Mapping aligned on a given map (generate a map from a sequence of RGBD images aligned witha given reference map recorded previously on the same environment, even with drastic changes)
    ```
    st_mapping-mapping_onref --help
    ```
- Deform a given map (deform a given map adapting it to changes visible through a new RGB-D sequence)
    ```
    st_mapping-deform_ref --help
    ```

**IMPORTANT**: to use the generic dataloader (the one used by default) you need to have a "params.yaml" in the dataset folder that describes the camera parameters. Please, refer to our [example file](https://github.com/PRBonn/spatio-temporal-mapping/blob/main/examples/params.yaml) or write your own dataloader. This feature will be updated soon to be more user-friendly.


## Citations and LICENSE
This project is free software made available under the MIT License. For details see the [LICENSE](https://github.com/PRBonn/spatio-temporal-mapping/blob/main/LICENSE) file.

If you use this project, please refer to our [paper on data association](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lobefaro2023iros.pdf) and [paper on plants deformation](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lobefaro2024iros.pdf):

```bibtex
@inproceedings{lobefaro2023iros,
  author = {L. Lobefaro and M.V.R. Malladi and O. Vysotska and T. Guadagnino and C. Stachniss},
  title = {{Estimating 4D Data Associations Towards Spatial-Temporal Mapping of Growing Plants for Agricultural Robots}},
  booktitle = iros,
  year = 2023,
  codeurl = {https://github.com/PRBonn/plants_temporal_matcher}
}
```
```bibtex
@inproceedings{lobefaro2024iros,
  author = {L. Lobefaro and M.V.R. Malladi and T. Guadagnino and C. Stachniss},
  title = {{Spatio-Temporal Consistent Mapping of Growing Plants for Agricultural Robots in the Wild}},
  booktitle = iros,
  year = 2024,
  codeurl = {https://github.com/PRBonn/spatio_temporal_mapping}
}
```

## Papers Results
As we decided to continue the development of this software after papers acceptance, we created a git branch so that researchers can consistently reproduce the results of the publication. To checkout at this branch, you can run the following command:

```
git checkout iros2024
```

The purpose of this software goes beyond the research done with the papers, we aim to push this research direction even more. For this reason, we strongly suggest you to use the version on the main branch because it allows better results and higher performances. The iros2024 branch exists only to ensure results reproducibility.


## Acknowledgement
The code structure of this software follows the same of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and some code is re-used from that repo. Please, if you use this software you should at least acknowledge also the work from KISS-ICP by giving a star on GitHub.

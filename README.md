# Spatio-Temporal Mapping  (IROS2024 version)

**IMPORTANT**: if you are on this branch it is because you want to reproduce the results of the [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lobefaro2024iros.pdf). If this is not the case we strongly suggest you to use the version on the main branch. This version is not supported and it is not user-friendly like the main one. It is here just to ensure the reproducibility of the research.

## Description
Mapping system for a glasshouse using the RGB-D cameras. It also allow to localize a robot in the same environment but weeks later, independently by the changes that the world undergo during that period of time. After that it is also possible to produce a spatial-temporal consistent 4D map of the environment, by obtaining the maps produced in the two session aligned.

## Installation 
**NOTE**: These instruction for installation are supported only for Ubuntu 22.04.
First install all the dependencies by typing:

```
./install_deps.sh
```

this will require your sudo password for package installation with apt package manager.

Then, you are ready to build the software by typing:

```
make build
```

This will also fetch [cilantro](https://github.com/kzampog/cilantro) library and build it locally.

## Paper result reproducibility
**NOTE**: In order to reproduce the same result of the [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lobefaro2024iros.pdf) you need access to the dataset. It is still not publicly available, for this reason we kindly ask you to send us an [email](mailto:llobefar@uni-bonn.de?subject=[GitHub]%20Data%20Request) to request the dataset.

Once you have the dataset downloaded and extracted you can run the script for evaluation:

```
./evaluation.sh /path/to/dataset/folder/
```

This will take a while. At the end of each experiment the results will be shown. Then you can press ENTER to go to the next experiment. The produced maps will be saved inside the dataset directories.
If you want to use the software for a single task with the possibility to visualize the produced maps, follow the instruction in the next section.

## How to use me
If you want to use this software for specific tasks you can follow the instruction reported here. Anyway, we strongly suggest you to switch to the main branch of this repo in this case, this branch is here just for results reproducibility.
If you want to produce a map from a RGB-D sequence, use the command:

```
make mapping dataset_main_folder=/path/to/dataset/folder/ ref_number=A row_number=C visualize=D
```

where A is the number corresponding to the recording that you want to use from the dataset (see dataset description), C is the number of glasshouse row that you want to map and D can be 1 or 0 if you want to visualize the final result or not.

If you want to produce a map from a RGB-D sequence aligned to a reference map, use the command:
```
make mapping_aligned dataset_main_folder=/path/to/dataset/folder/ ref_number=A query_number=B row_number=C visualize=D
```
where A is the number corresponding to the recording that you want to use from the dataset as reference (see dataset description), B is the recording to use for mapping on the reference, C is the number of glasshouse row that you want to map and D can be 1 or 0 if you want to visualize the final result or not. You need the map of the reference in order to work.

With the command:
```
make associate dataset_main_folder=/path/to/dataset/folder/ ref_number=A query_number=B row_number=C visualize=D
```
you can compute the visual matching between the two maps computed with the previous steps.

With the command:
```
make my_deform dataset_main_folder=/path/to/dataset/folder/ ref_number=A query_number=B row_number=C visualize=D 
```

you can run the deformation of the reference once the associations are computed.

To evaluate the result, just run:

```
make evaluate dataset_main_folder=/path/to/dataset/folder/ ref_number=A query_number=B row_number=C visualize=D
```


## Citations and Acknowledgements
If you use our code, please cite the corresponding [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lobefaro2024iros.pdf):

```bibtex
@inproceedings{lobefaro2024iros,
  author = {L. Lobefaro and M.V.R. Malladi and T. Guadagnino and C. Stachniss},
  title = {{Spatio-Temporal Consistent Mapping of Growing Plants for Agricultural Robots in the Wild}},
  booktitle = iros,
  year = 2024,
  codeurl = {https://github.com/PRBonn/spatio_temporal_mapping}
}
```

For our code we used [cilantro](https://github.com/kzampog/cilantro) library and we strongly inspired for non-rigid deformation from [this](https://github.com/rFalque/embedded_deformation) implementation of this [paper](https://people.inf.ethz.ch/~sumnerb/research/embdef/Sumner2007EDF.pdf). So please, if you like our work leave at least one star to those GitHub projects.


## LICENSE
This project is free software made available under the MIT License. For details see the [LICENSE](https://github.com/PRBonn/spatio-temporal-mapping/blob/main/LICENSE) file.

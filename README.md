![](assets/header_image.png)

[![Build Main](https://github.com/ika-rwth-aachen/acdc/actions/workflows/build.yml/badge.svg)](https://github.com/ika-rwth-aachen/acdc/actions/workflows/build.yml)


This repository belongs to the MOOC [Automated and Connected Driving Challenges (ACDC)](https://www.edx.org/course/automated-and-connected-driving-challenges). It contains ROS programming assignments. Enroll in the MOOC to get access to the solutions.


# Automated and Connected Driving Challenges

Repository of the official MOOC to learn [ROS](https://www.ros.org/) and algorithms used in automated driving.

Taught by the [Institute for Automotive Engineering (ika)](https://www.ika.rwth-aachen.de/), RWTH Aachen University.


## Instructions and exercises for this repository can be found in the [**Wiki**](https://github.com/ika-rwth-aachen/acdc/wiki)!


## Quick Start
Clone the repository __with__ the contained submodules:
```bash
git clone --recurse-submodules https://github.com/ika-rwth-aachen/acdc.git
```

Pull the docker image that is needed to run our environment
```bash
docker pull rwthika/acdc:latest
```

Launch the docker container in your terminal
```bash
acdc/docker $: ./run.sh
```



## License

Copyright (c) 2022, Institute for Automotive Engineering (ika), RWTH University

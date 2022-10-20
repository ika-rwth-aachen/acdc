# Object Fusion Packages

This folder contains two catkin packages:
- `object_fusion`: Source code in C++ of the core object fusion functionality. Does not contain a ROS node.
- `object_fusion_wrapper`: ROS-wrapper for usage of the `object_fusion` package as a ROS node.

## Modular structure
The fusion contains the following components (derived from `AbstractFusionModule`) in the given order:
- **ExistencePredictor**: Reduce existence probability of global objects over time.
- **ObjectRemover**: Remove global objects that have an existence probability below a certain threshold.
- **StatePredictor**: Predict the global objects's previous state to the current time. This uses the state transition matrix.
- **EgoMotionCompensation**: Transform the pose of global objects due to the motion of the ego-vehicle.
- **Matcher**: Assign / associate / match (synonyms here) measured objects to global objects.
- **StateFuser**: Kalman-Filter update step for fusing measured and global state.
- **HeadingAngleFuser**: Fuse measured and global heading angle by using its sin and cos values in a separate filter update step.
- **ExistenceFuser**: Fuse the existence probability of the measured and global object into one value.
- **ClassificationFuser**: Fuse the classification in case there is a classification mismatch between measured and global.
- **ObjectCreator**: create new global objects for measured objects without a global match. The initial variances of newly created global object are crucial for good performance.
- **MeasurementHistorian**: Insert which sensor has measured the global object over time.

More specific information about the internals is given in [./object_fusion/README.md](./object_fusion/README.md).

## Specify input sensors

- Add the input topic to the sources in [./object_fusion_wrapper/param/config_inout.yaml](./object_fusion_wrapper/param/config_inout.yaml).
- The input message format on this topic is `IkaObjectList.msg` (using `IkaObject.msg`). Both messages can be found in the `definitions` package.
- The input message needs to contain reasonable positive variance values for all measured state variables!
- State variables that are not measured must get a negative variance.

## Parameters

Upon wrapper node launch, several parameters are loaded from external `.yaml` files into the code. 
The parameters are explained in comments in the files themselves.
- [./object_fusion_wrapper/param/config_inout.yaml](./object_fusion_wrapper/param/config_inout.yaml) for input/output topics
- [./object_fusion_wrapper/param/fusion.yaml](./object_fusion_wrapper/param/fusion.yaml) for object-level data fusion parameters
- [./object_fusion_wrapper/param/kalman_filter.yaml](./object_fusion_wrapper/param/kalman_filter.yaml) for Kalman filter matrices

### Parameter tuning hints: 

You might need to carefully tune the parameters, especially the variances, to see the expected fusion results!
Therefore, it is important to get an understanding of the working principle of a Kalman filter with a simple vehicle model!

Influence of **high variances** on object association:

- Variances are in the denominator of the Mahalanobis distances used for object association.
- This means high variances result in small Mahalanobis distances between measured and global objects.
- Small Mahalanobis distances are likely to be smaller than the association threshold.
- This means high variances lead to an **association of distant objects**.
  - Variances *too high*: Objects can be associated that do not belong together, e.g. a driving car near a parking car.
  - Variances *too low*: A global object cannot even be associated with its own measurements, and a new global object is spawned for each measurement.

The influence of high variances on object association holds both for 
- variances of *measured object* state variables (specified in incoming `IkaObject.msg`)
- and for initial variance guesses of *global objects*, which have to be made for state variables that were not measured by the sensor who sees a global object for the first time (therefore specified in [fusion.yaml](./object_fusion_wrapper/param/fusion.yaml) config file).




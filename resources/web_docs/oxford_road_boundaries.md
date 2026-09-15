# Source: https://oxford-robotics-institute.github.io/road-boundaries-dataset/

Welcome to the Oxford Road Boundaries dataset, designed for training and testing machine-learning-based road-boundary detection and inference approaches.
We have hand-annotated two of the 10km long forays from the [Oxford Robotcar Dataset](https://robotcar-dataset.robots.ox.ac.uk/) and subsequently generated thousands of images with semi-annotated road boundary masks.
To boost the number of training samples, we used a vision-based localiser to project labels from the annotated datasets to other traversals at different times of data and weather conditions.
The Oxford Road Boundaries dataset contains 62605 labelled samples, of which 47639 samples are curated. Each of these samples contain both raw and classified masks for left and right lenses.
Our data contains images from a diverse set of scenarios such as straight roads, parked cars, and junctions.
Summary of data released
| Foray | # of frames ( A ) | # of frames ( C ) | % of occluded labels ( A ) | % of occluded labels ( C ) | Download size ( A ) | Download size ( C ) | Annotation | 
|---|---|---|---|---|---|---|---|
| 2015-05-26-13-59-22 | 22775 | 14890 | 31% | 29% | 63 GB | 41 GB | manual | 
| 2015-03-17-11-08-44 | 4869 | 3923 | 30% | 30% | 14 GB | 10 GB | automatic | 
| 2015-05-08-10-33-09 | 5519 | 4617 | 31% | 31% | 16 GB | 13 GB | automatic | 
| 2015-05-19-14-06-38 | 5424 | 4271 | 45% | 44% | 15 GB | 12 GB | automatic | 
| 2018-04-30-14-43-54 | 15153 | 13353 | 28% | 29% | 44 GB | 38 GB | manual | 
| 2019-01-10-11-46-21 | 4440 | 3392 | 39% | 39% | 12 GB | 10 GB | automatic | 
| 2019-01-10-12-32-52 | 4425 | 3193 | 40% | 38% | 12 GB | 8 GB | automatic | 
| Total | 62605 | 47639 | 33% | 32% | 176 GB | 132 GB | NA | 
A - all frames, C - curated frames
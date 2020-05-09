The following files are provided for testing in this directory:

#### test.py
driver for testing the controller.py and interlock.py; must be in the same directory as those files

#### lidar_data.yaml
example one-timestep dump of the Velodyne Puck VLP16 data output. maps row index to xyz data points

#### certificate_points.txt 
example text dump of the points chosen for one certificate

#### certificate_points_before_min_algorithm.txt
text dump of the certificate points when the controller does not run the minimum certificate algorithm

#### min_algorithm_timing_results.txt
Timing results from running the controller and interlock with and without the minimum certificate algorithm

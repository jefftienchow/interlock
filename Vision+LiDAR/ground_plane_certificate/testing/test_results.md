# Normal, working system (controller_and_interlock.py)
28.453991889953613s to get certificate
0.01143336296081543s for interlock response: True

# Faulty system 1: controller does not provide enough low-row points
23.10862898826599s to get certificate
0.0007088184356689453s for interlock response: False

# Faulty system 2: low-row points are not on the ground plane, or ground plane isn't satisfactorily ground-like
12.199167013168335s to get certificate
7.82012939453125e-05s for interlock response: False

### Note
These results are all nondeterministic, due to the random nature of the RANSAC algorithm.

Testing of more faulty systems is needed.
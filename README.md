

### Branches
 - Each branch corresponds to some prototype graph, and are not necessarily up-to-date with each other.
 - pilot: runs iSAM on the pilot0 dataset, generates GT at IMU frequency going forwards and backwards in a straight line.
 - pilot1: runs iSAM on the pilot1 dataset, generates GT at IMU frequency going in a square loop (~2.5 loops total).
 - pilot-LM: runs LM on the pilot0 dataset (helpful for debugging)

### Dependencies
 - Matplot++, I set it up following the guide [here](https://github.com/alandefreitas/matplotplusplus) under 'Install' for Ubuntu + GCC
 - NLohmann JSON, [github](https://github.com/nlohmann/json), I just cloned this repo into my /usr/include folder, and only needed to include the header at json/single_include.
 - I run this as a Visual Studio 2022 CMAKE project

### Data
 - Attached in UWBSLAM_pilot.zip

### Output Directories
 - I have a file stream open to dump a .dot file, I often use this when the graph crashes and I want to see the structure, I use 'xdot' on Ubuntu to visualize the structure.
 - I have a path for dumping the final estimated path to run error metrics with EVO, but I'm not using that at the moment.


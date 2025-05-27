

### Branches
 - Each branch corresponds to some prototype graph, and are not necessarily up-to-date with each other.
 - pilot: runs iSAM on the pilot0 dataset, generates GT at IMU frequency going in a straight line.
 - pilot1: runs iSAM on the pilot1 dataset, generates GT at IMU frequency going in a square loop (~2.5 loops total).
 - pilot-LM: runs LM on the pilot0 dataset (helpful for debugging)

### Dependencies
 - Matplot++, I set it up following the guide [here](https://github.com/alandefreitas/matplotplusplus) under 'Install' for Ubuntu + GCC
 - NLohmann JSON, [github](https://github.com/nlohmann/json), I think you only need to specify the header for this one, didn't need to build anything

### Data
 - Attached in UWBSLAM_pilot.zip

### Debug Dump Directory
 - I have a file stream open to dump a .dot file, I often use this when the 


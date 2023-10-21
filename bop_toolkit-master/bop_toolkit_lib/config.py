# Author: Tomas Hodan (hodantom@cmp.felk.cvut.cz)
# Center for Machine Perception, Czech Technical University in Prague

"""Configuration of the BOP Toolkit."""

import os


######## Basic ########

# Folder with the BOP datasets.
if 'BOP_PATH' in os.environ:
  datasets_path = os.environ['BOP_PATH']
else:
  datasets_path = r'/data/storage/BOP/'

# Folder with pose results to be evaluated.
# results_path = r'/home/qiaog/pose-est/feature_based_pose/scripts/results/'
# results_path = r'/root/pose-est/feature_based_pose/scripts/results/'
results_path = r'/home/qiaog/pose-est/oriented_features/python/oriented_features/pose_scoring_lightning/test_logs/bop_results'

# Folder for the calculated pose errors and performance scores.
# eval_path = r'/home/qiaog/pose-est/feature_based_pose/scripts/results/bop_eval/'
# eval_path = r'/root/pose-est/feature_based_pose/scripts/results/bop_eval/'
eval_path = r'/home/qiaog/pose-est/oriented_features/python/oriented_features/pose_scoring_lightning/test_logs/bop_results'

'''
./init_cpp_image_shell.sh
cd bop_toolkit
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/qiaog/pose-est/bop_renderer/osmesa-install/build/llvm-4.0.1.src/build/lib/
PYTHONPATH="./" python scripts/eval_bop19.py --renderer_type=cpp --result_filenames=ppf/opencv-ppf-icp_ycbv-test.csv
'''

######## Extended ########

# Folder for outputs (e.g. visualizations).
output_path = r'/path/to/output/folder'

# For offscreen C++ rendering: Path to the build folder of bop_renderer (github.com/thodan/bop_renderer).
bop_renderer_path = r'/home/qiaog/pose-est/bop_renderer/build'
# bop_renderer_path = r'/root/pose-est/bop_renderer/build'

# Executable of the MeshLab server.
meshlab_server_path = r'/path/to/meshlabserver.exe'

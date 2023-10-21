PYTHONPATH="./" python scripts/eval_bop19.py --renderer_type=cpp --result_filenames=$1 --targets_filename=test_targets_bop19_even.json &
PYTHONPATH="./" python scripts/eval_bop19.py --renderer_type=cpp --result_filenames=$1 --targets_filename=test_targets_bop19_odd.json &

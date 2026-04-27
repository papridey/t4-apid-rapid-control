%EXAMPLE_RUN_APID_BANDLOCK_SMOOTH Minimal APID example.

this_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(fileparts(this_dir));
addpath(fullfile(repo_dir, 'apid'));

OUT = run_apid_bandlock_smooth(25, struct('make_plots', true, 'verbose', true));

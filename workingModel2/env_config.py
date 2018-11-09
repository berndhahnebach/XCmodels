# -*- coding: utf-8 -*-
from postprocess.config import default_config

import sys
sys.path.append('..')
from xcmodelsdir import xcmodelsdir
# print(xcmodelsdir)


# Default configuration of environment variables.
cfg=default_config.envConfig(
    language = 'en',
    intForcPath = xcmodelsdir + 'workingModel/results/internalForces/',
    verifPath = xcmodelsdir + 'workingModel2/results/verifications/',
    annexPath = xcmodelsdir + 'workingModel2/annex/',
    grWidth = '120mm'
)
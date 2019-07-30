#!/usr/bin/env python
import time
import numpy as np

import christine_hwi_ext

hwi = christine_hwi_ext.HWI()
hwi.start()
hwi.write()
time.sleep(1.)
hwi.shutdown()


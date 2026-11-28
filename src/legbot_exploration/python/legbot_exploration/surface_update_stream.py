"""Versioned measured-coverage deltas; never an authoritative planning map."""
import numpy as np


class SurfaceUpdateStream:
    def __init__(self, coverage):
        self.coverage=coverage
        self.version=None
        self.epoch=None
        self.synchronized=False
        self.gaps=0

    def apply(self, value):
        version=value['version'];epoch=value['epoch']
        if self.version is not None and epoch==self.epoch and version<self.version:
            return False
        full=bool(value.get('full'))
        if not full and (not self.synchronized or epoch!=self.epoch or
                         value['base_version']!=self.version):
            self.synchronized=False;self.gaps+=1;return False
        data=np.asarray(value['coverage'],dtype=float).reshape(-1,4)
        if len(data):self.coverage.observe(data[:,:3],data[:,3].astype(np.uint32))
        self.version=version;self.epoch=epoch;self.synchronized=True
        return True

    def covers(self, version):
        return self.synchronized and self.version is not None and self.version>=version

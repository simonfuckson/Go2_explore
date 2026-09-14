#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ground_clearing_adapter import expand_support_cells

class SupportClearing(unittest.TestCase):
    def test_preserves_measured_cells_and_fills_five_cm_resolution(self):
        p=np.array([[.475,.125,-.5],[.625,.125,-.49]])
        result=expand_support_cells(p,.15,(-5.,-4.),.05,.35,5.)
        keys=np.floor((result[:,:2]-[-5.,-4.])/.15).astype(int)
        source=np.floor((p[:,:2]-[-5.,-4.])/.15).astype(int)
        self.assertEqual(set(map(tuple,keys)),set(map(tuple,source)))
        self.assertEqual(len(result),72)
        self.assertEqual(set(result[:,2]),{-.5,-.49})
        self.assertGreater(len(np.unique(np.floor(result[:,:2]/.05),axis=0)),len(p))
    def test_empty_unknown_and_raw_returns_are_not_invented_support(self):
        self.assertEqual(expand_support_cells([], .15,(-5.,-4.),.05,.35,5.).shape,(0,3))
        with self.assertRaises(ValueError):expand_support_cells([[.51,.11,.2]],.15,(-5.,-4.),.05,.35,5.)
    def test_range_clipping_preserves_mount_blind_area(self):
        result=expand_support_cells([[.325,.125,-.5]],.15,(-5.,-4.),.05,.35,.4)
        self.assertTrue(np.all(np.linalg.norm(result[:,:2],axis=1)>=.35))
        self.assertTrue(np.all(np.linalg.norm(result[:,:2],axis=1)<=.4))

if __name__=='__main__':unittest.main()

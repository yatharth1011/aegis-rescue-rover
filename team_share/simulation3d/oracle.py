"""Ground-truth detector (debug only): bbox of the token from MuJoCo segmentation."""
import numpy as np


class Oracle:
    def __init__(self):
        self.world = None

    def __call__(self, img):
        w = self.world
        seg = w.render(segmentation=True)
        gids = {w.m.geom("token_g").id, w.m.geom("token_band").id}
        mask = np.isin(seg[..., 0], list(gids)) & (seg[..., 1] == 5)
        if mask.sum() < 12:
            return []
        ys, xs = np.nonzero(mask)
        return [(xs.min(), ys.min(), xs.max(), ys.max(), 0.99)]

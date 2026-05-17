import os

import cv2
import torch
from omegaconf import OmegaConf

PT_WEIGHT_NAME = 'model.pth'
ONNX_CFG_NAME = 'onnx.yaml'
FEATURE_ENGINE_NAME = 'feature_runner.engine'
POST_ENGINE_NAME = 'post_runner.engine'


class FastFoundationTrtAPI:
    def __init__(self, weight_dir, target_size=(480, 640), device='cuda'):
        from fastfoundation.fastfoundation import ensure_foundation_stereo_importable

        ensure_foundation_stereo_importable()
        from core.foundation_stereo import TrtRunner

        self.device = torch.device(device)
        self.target_h, self.target_w = target_size

        yaml_path = os.path.join(weight_dir, ONNX_CFG_NAME)
        self.args = OmegaConf.load(yaml_path)
        if not hasattr(self.args, 'image_size'):
            self.args.image_size = [self.target_h, self.target_w]
        else:
            self.target_h = int(self.args.image_size[0])
            self.target_w = int(self.args.image_size[1])

        feature_engine = os.path.join(weight_dir, FEATURE_ENGINE_NAME)
        post_engine = os.path.join(weight_dir, POST_ENGINE_NAME)
        self.model = TrtRunner(self.args, feature_engine, post_engine)

    @torch.no_grad()
    def predict(self, left_bgr, right_bgr):
        h_orig, w_orig = left_bgr.shape[:2]
        fx = self.target_w / w_orig
        fy = self.target_h / h_orig

        if fx != 1.0 or fy != 1.0:
            left = cv2.resize(left_bgr, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
            right = cv2.resize(right_bgr, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
        else:
            left, right = left_bgr, right_bgr

        cam1 = torch.from_numpy(left).to(self.device).float().permute(2, 0, 1)[None]
        cam2 = torch.from_numpy(right).to(self.device).float().permute(2, 0, 1)[None]

        disp = self.model.forward(cam1, cam2)
        disp = disp.squeeze().cpu().numpy()
        disp_restored = disp * (1.0 / fx)

        if fx != 1.0 or fy != 1.0:
            disp_restored = cv2.resize(disp_restored, (w_orig, h_orig), interpolation=cv2.INTER_LINEAR)

        return disp_restored.clip(0, None)


class FastFoundationPtAPI:
    def __init__(self, weight_dir, target_size=(480, 640), device='cuda'):
        from fastfoundation.fastfoundation import ensure_foundation_stereo_importable

        ensure_foundation_stereo_importable()
        from core.utils.utils import InputPadder

        self._InputPadder = InputPadder
        self.device = torch.device(device)
        self.target_h, self.target_w = target_size

        yaml_path = os.path.join(weight_dir, ONNX_CFG_NAME)
        cfg = OmegaConf.load(yaml_path)
        if hasattr(cfg, 'image_size'):
            self.target_h = int(cfg.image_size[0])
            self.target_w = int(cfg.image_size[1])

        pt_path = os.path.join(weight_dir, PT_WEIGHT_NAME)
        if not os.path.isfile(pt_path):
            raise FileNotFoundError(
                f'PyTorch backend expects `{PT_WEIGHT_NAME}` under weight_dir, missing: {pt_path}'
            )
        try:
            self.model = torch.load(pt_path, map_location='cpu', weights_only=False)
        except TypeError:
            self.model = torch.load(pt_path, map_location='cpu')
        self.model.args.valid_iters = int(cfg.valid_iters)
        self.model.args.max_disp = int(cfg.max_disp)
        self.model.to(self.device).eval()

        try:
            from Utils import AMP_DTYPE
            self._amp_dtype = AMP_DTYPE
        except ImportError:
            self._amp_dtype = torch.float16

    @torch.no_grad()
    def predict(self, left_bgr, right_bgr):
        h_orig, w_orig = left_bgr.shape[:2]
        fx = self.target_w / w_orig
        fy = self.target_h / h_orig

        if fx != 1.0 or fy != 1.0:
            left = cv2.resize(left_bgr, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
            right = cv2.resize(right_bgr, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
        else:
            left, right = left_bgr, right_bgr

        left_t = torch.from_numpy(left).to(self.device).float()[None].permute(0, 3, 1, 2)
        right_t = torch.from_numpy(right).to(self.device).float()[None].permute(0, 3, 1, 2)
        padder = self._InputPadder(left_t.shape, divis_by=32, force_square=False)
        left_t, right_t = padder.pad(left_t, right_t)

        use_amp = bool(getattr(self.model.args, 'mixed_precision', True))
        with torch.amp.autocast('cuda', enabled=use_amp, dtype=self._amp_dtype):
            disp = self.model.forward(
                left_t, right_t,
                iters=int(self.model.args.valid_iters),
                test_mode=True,
                optimize_build_volume='pytorch1',
            )
        disp = padder.unpad(disp.float())
        disp_np = disp.squeeze().cpu().numpy()
        disp_restored = disp_np * (1.0 / fx)

        if fx != 1.0 or fy != 1.0:
            disp_restored = cv2.resize(disp_restored, (w_orig, h_orig), interpolation=cv2.INTER_LINEAR)

        return disp_restored.clip(0, None)


def create_predictor(inference_backend, weight_dir, device):
    backend = (inference_backend or 'engine').strip().lower()
    if backend == 'engine':
        return FastFoundationTrtAPI(weight_dir=weight_dir, device=device)
    if backend == 'pt':
        return FastFoundationPtAPI(weight_dir=weight_dir, device=device)
    raise ValueError(
        f"Unsupported inference_backend '{inference_backend}'. Use 'engine' or 'pt'."
    )

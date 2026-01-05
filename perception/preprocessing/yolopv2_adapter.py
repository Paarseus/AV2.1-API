"""
YOLOPv2 Adapter - Official Implementation
Multi-task inference: object detection, drivable area, and lane line segmentation

Usage:
    yolo = YOLOPv2Adapter("yolopv2.pt")
    output = yolo.infer(bgr_image)
"""

import torch
import torchvision
import cv2
import numpy as np
import time
import os
from typing import Tuple

from ..core_types import YOLOPv2Output


# ============================================================================
# Official YOLOPv2 Utility Functions
# ============================================================================

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    """Resize and pad image while maintaining aspect ratio"""
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)
    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)


def _make_grid(nx=20, ny=20):
    """Create grid for YOLO predictions"""
    yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
    return torch.stack((xv, yv), 2).view((1, 1, ny, nx, 2)).float()


def split_for_trace_model(pred=None, anchor_grid=None):
    """Decode predictions from torch.jit.trace model"""
    z = []
    st = [8, 16, 32]
    for i in range(3):
        bs, _, ny, nx = pred[i].shape
        pred[i] = pred[i].view(bs, 3, 85, ny, nx).permute(0, 1, 3, 4, 2).contiguous()
        y = pred[i].sigmoid()
        gr = _make_grid(nx, ny).to(pred[i].device)
        y[..., 0:2] = (y[..., 0:2] * 2. - 0.5 + gr) * st[i]
        y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * anchor_grid[i]
        z.append(y.view(bs, -1, 85))
    pred = torch.cat(z, 1)
    return pred


def xywh2xyxy(x):
    """Convert boxes from [x, y, w, h] to [x1, y1, x2, y2]"""
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def clip_coords(boxes, img_shape):
    """Clip bounding boxes to image boundaries"""
    boxes[:, 0].clamp_(0, img_shape[1])
    boxes[:, 1].clamp_(0, img_shape[0])
    boxes[:, 2].clamp_(0, img_shape[1])
    boxes[:, 3].clamp_(0, img_shape[0])


def box_iou(box1, box2):
    """Calculate IoU between two sets of boxes"""
    def box_area(box):
        return (box[2] - box[0]) * (box[3] - box[1])

    area1 = box_area(box1.T)
    area2 = box_area(box2.T)
    inter = (torch.min(box1[:, None, 2:], box2[:, 2:]) - torch.max(box1[:, None, :2], box2[:, :2])).clamp(0).prod(2)
    return inter / (area1[:, None] + area2 - inter)


def non_max_suppression(prediction, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False, multi_label=False,
                        labels=()):
    """Non-Maximum Suppression"""
    nc = prediction.shape[2] - 5
    xc = prediction[..., 4] > conf_thres
    max_det = 300
    max_nms = 30000
    time_limit = 10.0
    redundant = True
    multi_label &= nc > 1
    merge = False
    t = time.time()
    output = [torch.zeros((0, 6), device=prediction.device)] * prediction.shape[0]

    for xi, x in enumerate(prediction):
        x = x[xc[xi]]
        if labels and len(labels[xi]):
            l = labels[xi]
            v = torch.zeros((len(l), nc + 5), device=x.device)
            v[:, :4] = l[:, 1:5]
            v[:, 4] = 1.0
            v[range(len(l)), l[:, 0].long() + 5] = 1.0
            x = torch.cat((x, v), 0)
        if not x.shape[0]:
            continue

        x[:, 5:] *= x[:, 4:5]
        box = xywh2xyxy(x[:, :4])

        if multi_label:
            i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
            x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
        else:
            conf, j = x[:, 5:].max(1, keepdim=True)
            x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

        if classes is not None:
            x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

        n = x.shape[0]
        if not n:
            continue
        elif n > max_nms:
            x = x[x[:, 4].argsort(descending=True)[:max_nms]]

        c = x[:, 5:6] * (0 if agnostic else 4096)
        boxes, scores = x[:, :4] + c, x[:, 4]
        i = torchvision.ops.nms(boxes, scores, iou_thres)
        if i.shape[0] > max_det:
            i = i[:max_det]

        if merge and (1 < n < 3E3):
            iou = box_iou(boxes[i], boxes) > iou_thres
            weights = iou * scores[None]
            x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)
            if redundant:
                i = i[iou.sum(1) > 1]

        output[xi] = x[i]
        if (time.time() - t) > time_limit:
            print(f'WARNING: NMS time limit {time_limit}s exceeded')
            break

    return output


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    """Rescale coordinates from img1_shape to img0_shape"""
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]
    coords[:, [0, 2]] -= pad[0]
    coords[:, [1, 3]] -= pad[1]
    coords[:, :4] /= gain
    clip_coords(coords, img0_shape)
    return coords


def driving_area_mask(seg):
    """Extract drivable area mask from segmentation output"""
    da_predict = seg[:, :, 12:372, :]
    da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=2, mode='bilinear')
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
    return da_seg_mask


def lane_line_mask(ll):
    """Extract lane line mask from segmentation output"""
    ll_predict = ll[:, :, 12:372, :]
    ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=2, mode='bilinear')
    ll_seg_mask = torch.round(ll_seg_mask).squeeze(1)
    ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
    return ll_seg_mask


# ============================================================================
# YOLOPv2 Adapter
# ============================================================================

class YOLOPv2Adapter:
    """YOLOPv2 inference wrapper"""

    def __init__(self, weights=None, device="cuda", img_size=640, conf_thres=0.3, iou_thres=0.45, stride=32):
        self.img_size = img_size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.stride = stride

        if weights is None:
            weights = os.path.join(os.path.dirname(__file__), "yolopv2.pt")

        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.half = self.device.type == 'cuda'

        self.model = torch.jit.load(weights, map_location=self.device)
        if self.half:
            self.model.half()
        self.model.eval()

        print(f"[YOLOPv2] Loaded on {self.device}")

    def _preprocess(self, image: np.ndarray) -> Tuple[torch.Tensor, Tuple[int, int], Tuple]:
        """Preprocess image: letterbox resize, BGR->RGB, normalize to [0,1]"""
        self.orig_shape = image.shape[:2]

        img, ratio, pad = letterbox(image, new_shape=self.img_size, stride=self.stride, auto=False)
        self.ratio_pad = (ratio, pad)

        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.0

        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        return img, self.orig_shape, self.ratio_pad

    def _postprocess_detections(self, pred, anchor_grid) -> np.ndarray:
        """Decode predictions and apply NMS"""
        pred = split_for_trace_model(pred, anchor_grid)
        pred = non_max_suppression(pred, conf_thres=self.conf_thres, iou_thres=self.iou_thres)

        det = pred[0]
        if len(det) == 0:
            return np.zeros((0, 6))

        det[:, :4] = scale_coords((self.img_size, self.img_size), det[:, :4], self.orig_shape,
                                   ratio_pad=self.ratio_pad)
        return det.cpu().numpy()

    def _postprocess_segmentation(self, seg, ll) -> Tuple[np.ndarray, np.ndarray]:
        """Extract drivable area and lane line masks"""
        da_mask = driving_area_mask(seg)
        ll_mask = lane_line_mask(ll)

        # Resize masks to original image dimensions
        da_mask = cv2.resize(da_mask, (self.orig_shape[1], self.orig_shape[0]), interpolation=cv2.INTER_NEAREST)
        ll_mask = cv2.resize(ll_mask, (self.orig_shape[1], self.orig_shape[0]), interpolation=cv2.INTER_NEAREST)

        da_mask = (da_mask * 255).astype(np.uint8)
        ll_mask = (ll_mask * 255).astype(np.uint8)

        return da_mask, ll_mask

    @torch.no_grad()
    def infer(self, image: np.ndarray, camera_id: str = "camera") -> YOLOPv2Output:
        """
        Run YOLOPv2 inference

        Args:
            image: BGR image (H, W, 3) uint8
            camera_id: Camera identifier

        Returns:
            YOLOPv2Output with detections and masks
        """
        timestamp = time.time()

        img_tensor, orig_shape, ratio_pad = self._preprocess(image)
        [pred, anchor_grid], seg, ll = self.model(img_tensor)

        detections = self._postprocess_detections(pred, anchor_grid)
        drivable_area, lane_lines = self._postprocess_segmentation(seg, ll)

        return YOLOPv2Output(
            timestamp=timestamp,
            drivable_area=drivable_area,
            lane_lines=lane_lines,
            detections=detections,
            image_shape=orig_shape,
            camera_id=camera_id
        )

#!/usr/bin/env python3
"""
Official YOLOPv2 demo.py - modified for RealSense camera
Original: https://github.com/CAIC-AD/YOLOPv2/blob/main/demo.py
Only change: RealSense camera instead of LoadImages
"""

import argparse
import time
from pathlib import Path
import cv2
import torch
import numpy as np

import pyrealsense2 as rs

from utils.utils import \
    time_synchronized,select_device, increment_path,\
    scale_coords,xyxy2xywh,non_max_suppression,split_for_trace_model,\
    driving_area_mask,lane_line_mask,plot_one_box,show_seg_result,\
    AverageMeter, letterbox


def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='data/weights/yolopv2.pt', help='model.pt path(s)')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.3, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    return parser


def detect():
    weights, imgsz = opt.weights, opt.img_size

    inf_time = AverageMeter()
    nms_time = AverageMeter()

    # Load model
    stride = 32
    device = select_device(opt.device)
    model = torch.jit.load(weights, map_location='cpu')
    half = device.type != 'cpu'
    model = model.to(device)

    if half:
        model.half()
    model.eval()
    print(f"Model loaded on {device}")

    # === REALSENSE CAMERA (instead of LoadImages) ===
    pipeline = rs.pipeline()
    config = rs.config()
    # 1280x720 like official demo's LoadImages resize
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    print("RealSense camera started at 1280x720")

    # Warmup
    for _ in range(5):
        pipeline.wait_for_frames(timeout_ms=5000)

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))

    t0 = time.time()
    frame_count = 0

    try:
        while True:
            # === GET FRAME FROM REALSENSE ===
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # im0s is 1280x720 (same as official LoadImages resize)
            im0s = np.asanyarray(color_frame.get_data())

            # === EXACTLY LIKE OFFICIAL DEMO FROM HERE ===
            # Letterbox
            img = letterbox(im0s, imgsz, stride=stride)[0]
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
            img = np.ascontiguousarray(img)

            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()
            img /= 255.0

            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

            tw1 = time_synchronized()
            pred = split_for_trace_model(pred, anchor_grid)
            tw2 = time_synchronized()

            # Apply NMS
            t3 = time_synchronized()
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
            t4 = time_synchronized()

            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)

            # Process detections
            for i, det in enumerate(pred):
                im0 = im0s.copy()

                if len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                    for *xyxy, conf, cls in reversed(det):
                        plot_one_box(xyxy, im0, line_thickness=3)

                show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)

                # Display
                cv2.imshow("Official YOLOPv2 Demo + RealSense", im0)

            frame_count += 1
            inf_time.update(t2-t1, img.size(0))
            nms_time.update(t4-t3, img.size(0))

            if frame_count % 10 == 0:
                print(f'Frame {frame_count} | inf: {inf_time.avg*1000:.1f}ms | nms: {nms_time.avg*1000:.1f}ms')

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f'Done. {frame_count} frames in {time.time() - t0:.1f}s')


if __name__ == '__main__':
    opt = make_parser().parse_args()
    print(opt)

    with torch.no_grad():
        detect()

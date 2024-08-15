# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

# sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# import argparse
import torch
import onnx
import onnxsim
import yaml
# from pathlib import Path
from torch.ao.quantization import fuse_modules

from general.config.config import cfg, cfg_from_yaml_file
from object_detection.detectors3d import build_network
from general.utilities import common_utils
# from general.datasets.dataset_template import DatasetTemplate
from object_detection.datasets.dummy.dummy_dataset import DummyDataset
# from general.utilities.data_utils import load_data_to_gpu
# from object_detection.detectors3d.centerpoint_pillar.onnx_utils import pillarscatter_surgeon

import onnx
import numpy as np
import onnx_graphsurgeon as gs


@gs.Graph.register()
def replace_with_clip(self, inputs, outputs, grid_y_size, grid_x_size):
    for inp in inputs:
        inp.outputs.clear()

    for out in outputs:
        out.inputs.clear()

    op_attrs = dict()
    op_attrs["dense_shape"] = np.array([grid_y_size, grid_x_size])

    return self.layer(name="PPScatter_0", op="PPScatterPlugin",
                      inputs=inputs, outputs=outputs, attrs=op_attrs)


def pillarscatter_surgeon(onnx_model):
    graph = gs.import_onnx(onnx_model)
    grapth_tensors = graph.tensors()

    # Input shapes
    voxel_idxs_shape = grapth_tensors["voxel_idxs"].shape
    voxel_num_shape = grapth_tensors["voxel_num"].shape

    # Pillar Feature Net Input
    input_new_node = [node for node in graph.nodes if node.op == "MatMul"][0]
    input_new_shape = input_new_node.inputs[0].shape

    # PillarScatter Input
    voxels = gs.Variable(name="voxels", dtype=np.float32, shape=input_new_shape)
    voxels_idxs = gs.Variable(name="voxel_idxs", dtype=np.int32, shape=voxel_idxs_shape)
    voxels_num = gs.Variable(name="voxel_num", dtype=np.int32, shape=voxel_num_shape)

    pillar_feature_input = [node for node in graph.nodes if node.name == '/vfe/pfn_layers.1/ReduceMax'][0]
    pillar_feature_input.attrs["keepdims"] = 0
    output_tensor = graph.tensors()[pillar_feature_input.outputs[0].name]
    output_tensor.shape.remove(1)

    # Graph surgery
    conv_op = [node for node in graph.nodes if node.op == "Conv"][0]
    # graph.inputs.append(voxels_num)
    inputs = [pillar_feature_input.outputs[0], voxels_idxs, voxels_num]
    outputs = [conv_op.inputs[0]]
    grid_y_size, grid_x_size = conv_op.inputs[0].shape[2:]

    graph.replace_with_clip(inputs, outputs, grid_y_size, grid_x_size)
    graph.cleanup().toposort()

    graph.inputs = [voxels, voxels_idxs, voxels_num]
    input_new_node.inputs[0] = voxels
    graph.outputs = [grapth_tensors[output.name] for output in graph.outputs]

    graph.cleanup().toposort()
    modified_model = gs.export_onnx(graph)

    return modified_model


def convert_onnx(args, onnx_raw_path, onnx_sim_path):
    logger = common_utils.create_logger()
    logger.info("------ Convert OpenPCDet model to ONNX ------")
    dataset = DummyDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
        root_path=args.data_path, logger=logger
    )

    # Build the model
    cfg.MODEL.DENSE_HEAD.POST_PROCESSING.EXPORT_ONNX = True
    cfg.MODEL.POST_PROCESSING.EXPORT_ONNX = True
    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES),
                          dataset=dataset)
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()

    # Fuse the ConvTranspose2d + BN layers
    for deblock in model.backbone_2d.deblocks:
        if deblock[0].__class__.__name__ == "ConvTranspose2d":
            fuse_modules(deblock, [['0', '1']], inplace=True)

    # Prepare the input
    with torch.no_grad():
        for item in cfg.DATA_CONFIG.DATA_PROCESSOR:
            if item.NAME == "transform_points_to_voxels":
                # max_voxels = item.MAX_NUMBER_OF_VOXELS['train']
                max_voxels = args.max_voxels
                max_points_per_voxel = item.MAX_POINTS_PER_VOXEL
        num_point_features = 4

        dummy_voxels = torch.zeros(
            (max_voxels, max_points_per_voxel, num_point_features),
            dtype=torch.float32, device='cuda'
        )
        dummy_voxel_num = torch.zeros((1,), dtype=torch.int32, device='cuda')
        dummy_voxel_idxs = torch.zeros((max_voxels, 4), dtype=torch.int32, device='cuda')

        # Export the model to ONNX
        dummy_input = ({'voxels': dummy_voxels,
                        'voxel_num_points': dummy_voxel_num,
                        'voxel_coords': dummy_voxel_idxs,
                        'batch_size': 1}, {})
        input_names = ['voxels', 'voxel_num', 'voxel_idxs']
        output_names = list(cfg.MODEL.DENSE_HEAD.SEPARATE_HEAD_CFG.HEAD_DICT.keys()) + ["score", "label"]
        torch.onnx.export(model,
                          dummy_input,
                          onnx_raw_path,
                          export_params=True,
                          opset_version=14,
                          do_constant_folding=True,
                          keep_initializers_as_inputs=True,
                          input_names=input_names,
                          output_names=output_names)
        onnx_raw = onnx.load(onnx_raw_path)
        onnx_sim, _ = onnxsim.simplify(onnx_raw)
        onnx.save(onnx_sim, onnx_sim_path)

    logger.info("Model exported to model.onnx")


def export_config(save_dir, max_voxels=25000, max_points=200000, score_threshold=0.5, num_voxel_feature=10):
    """

    :param save_dir:
    :param max_voxels:
    :param max_points:
    :param score_threshold:
    :param num_voxel_feature:
    :return:
    """

    centerpoint_dict = {}
    centerpoint_dict['max_points'] = max_points
    centerpoint_dict['pub'] = "/centerpoint/boxes"
    centerpoint_dict['sub'] = "/lidar/concatenated/pointcloud"
    centerpoint_dict['score_threshold'] = score_threshold

    voxelization_dict = {'min_range': {},
                         'max_range': {},
                         'voxel_size': {}}
    for item in cfg.DATA_CONFIG.DATA_PROCESSOR:
        if item.NAME == "transform_points_to_voxels":
            voxelization_config = item
    voxelization_dict['min_range']['x'] = cfg.DATA_CONFIG.POINT_CLOUD_RANGE[0]
    voxelization_dict['min_range']['y'] = cfg.DATA_CONFIG.POINT_CLOUD_RANGE[1]
    voxelization_dict['min_range']['z'] = cfg.DATA_CONFIG.POINT_CLOUD_RANGE[2]
    voxelization_dict['max_range']['x'] = cfg.DATA_CONFIG.POINT_CLOUD_RANGE[3]
    voxelization_dict['max_range']['y'] = cfg.DATA_CONFIG.POINT_CLOUD_RANGE[4]
    voxelization_dict['max_range']['z'] = cfg.DATA_CONFIG.POINT_CLOUD_RANGE[5]
    voxelization_dict['voxel_size']['x'] = voxelization_config.VOXEL_SIZE[0]
    voxelization_dict['voxel_size']['y'] = voxelization_config.VOXEL_SIZE[1]
    voxelization_dict['voxel_size']['z'] = voxelization_config.VOXEL_SIZE[2]
    # voxelization_dict['max_voxels'] = args.max_voxels
    voxelization_dict['max_voxels'] = max_voxels
    voxelization_dict['max_points_per_voxel'] = voxelization_config.MAX_POINTS_PER_VOXEL
    voxelization_dict['num_feature'] = len(cfg.DATA_CONFIG.POINT_FEATURE_ENCODING.used_feature_list)
    voxelization_dict['num_voxel_feature'] = num_voxel_feature

    postprocess_dict = {'nms': {}}
    postprocess_config = cfg.MODEL.DENSE_HEAD.POST_PROCESSING
    postprocess_dict['iou'] = postprocess_config.IOU_RECTIFIER
    postprocess_dict['nms']['pre_max'] = postprocess_config.NMS_CONFIG.NMS_PRE_MAXSIZE
    postprocess_dict['nms']['post_max'] = postprocess_config.NMS_CONFIG.NMS_POST_MAXSIZE
    postprocess_dict['nms']['iou_threshold'] = postprocess_config.NMS_CONFIG.NMS_THRESH
    postprocess_dict['out_size_factor'] = cfg.MODEL.DENSE_HEAD.TARGET_ASSIGNER_CONFIG.FEATURE_MAP_STRIDE
    postprocess_dict['feature_x_size'] = round(
        (voxelization_dict['max_range']['x'] - voxelization_dict['min_range']['x']) / voxelization_dict['voxel_size'][
            'x'] / postprocess_dict['out_size_factor'])
    postprocess_dict['feature_y_size'] = round(
        (voxelization_dict['max_range']['y'] - voxelization_dict['min_range']['y']) / voxelization_dict['voxel_size'][
            'y'] / postprocess_dict['out_size_factor'])
    postprocess_dict['pillar_x_size'] = voxelization_dict['voxel_size']['x']
    postprocess_dict['pillar_y_size'] = voxelization_dict['voxel_size']['y']
    postprocess_dict['min_x_range'] = voxelization_dict['min_range']['x']
    postprocess_dict['min_y_range'] = voxelization_dict['min_range']['y']

    # save_path = Path("../") / "config.yaml"
    save_path = save_dir / "config.yaml"

    cfg_output = {'centerpoint': centerpoint_dict,
                  'voxelization': voxelization_dict,
                  'postprocess': postprocess_dict}
    with open(save_path, 'w') as f:
        yaml.dump(cfg_output, f, default_flow_style=False)
    print(f"Config exported to {save_path}")

from general.datasets.dataset_template import DatasetTemplate


class DummyDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None):
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )

    def __len__(self):
        return 0

    def __getitem__(self, index):
        return {}

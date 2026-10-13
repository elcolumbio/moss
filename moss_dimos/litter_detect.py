"""Litter detection: dimOS's object scene registration on the nose camera,
with a YOLO trained on litter instead of the open-vocabulary model it ships
with.

`LitterDetection` is dimOS's `ObjectSceneRegistrationModule` with one change:
`start()` builds a `Yolo2DDetector` on a weights file of ours (`weights`,
an absolute path; MOSS_LITTER_WEIGHTS in the blueprint) instead of the
YOLOE the module downloads. Everything else is the module's: colour and
depth frames aligned in time, each 2D box lifted to a 3D object through the
depth image and the camera's transform to `target_frame` (the world frame
of the odometry), objects confirmed after `min_detections_for_permanent`
sightings and published on `objects`, which the mission reads.

The weights are not in this repository: a small YOLO trained on waste
images, five material classes (glass, metal, paper, plastic, waste), from
a public repository under the MIT licence; docs/litter.md says where.
Confidence and IoU are the detector's own (0.5 / 0.6).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from dimos.core.core import rpc
from dimos.perception.experimental import object_scene_registration as osr
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule

WEIGHTS_ENV = "MOSS_LITTER_WEIGHTS"
WEIGHTS_DEFAULT = "~/.local/state/moss/models/litter.pt"
LITTER_CLASSES_ENV = "MOSS_LITTER_CLASSES"                     # comma-separated class names the mission picks up
LITTER_CLASSES_DEFAULT = "Glass,Metal,Paper,Plastic,Waste"      # the five-material model's names
DETECTOR_DEVICE_ENV = "MOSS_LITTER_DEVICE"                     # cpu, cuda; unset = the detector's own choice


def weights_path(env: dict | None = None) -> Path:
    env = os.environ if env is None else env
    return Path(env.get(WEIGHTS_ENV, "").strip() or WEIGHTS_DEFAULT).expanduser()


def litter_classes(env: dict | None = None) -> set[str]:
    env = os.environ if env is None else env
    raw = env.get(LITTER_CLASSES_ENV, "").strip() or LITTER_CLASSES_DEFAULT
    return {c.strip() for c in raw.split(",") if c.strip()}


class LitterDetection(ObjectSceneRegistrationModule):
    """Object scene registration with our YOLO weights."""

    def __init__(self, weights: str | os.PathLike | None = None, device: str | None = None,
                 **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._weights = Path(weights).expanduser() if weights else weights_path()
        self._device = device or os.environ.get(DETECTOR_DEVICE_ENV, "").strip() or None

    @rpc
    def start(self) -> None:
        """The module's start with our detector in place of the YOLOE."""
        from dimos.perception.detection.detectors.yolo import Yolo2DDetector

        if not self._weights.is_file():
            raise FileNotFoundError(f"litter detection: no weights at {self._weights} ({WEIGHTS_ENV})")
        osr.Module.start(self)
        # an absolute model_path makes the detector skip its LFS lookup
        self._detector = Yolo2DDetector(model_path=str(self._weights.parent), model_name=self._weights.name,
                                        device=self._device)
        self.camera_info.subscribe(lambda msg: setattr(self, "_camera_info", msg))
        aligned = osr.align_timestamped(self.color_image.observable(), self.depth_image.observable(),
                                        buffer_size=2.0, match_tolerance=0.1)
        osr.backpressure(aligned).subscribe(self._on_aligned_frames)
        osr.logger.info(f"litter detection started: {self._weights.name}, device {self._device or 'auto'}, "
                        f"target frame {self._target_frame}")

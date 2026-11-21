import csv
from pathlib import Path

import mujoco
import numpy as np

from .t2_safety import JOINT_NAMES, command_pose, valid_pose


MODEL_DIR = Path("/opt/go1_model")
MAX_PENETRATION_M = 0.001


class Go1Collision:
    def __init__(self, model_dir=MODEL_DIR):
        self.model = mujoco.MjModel.from_xml_path(str(model_dir / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        # У qpos первые семь координат принадлежат корпусу. Индексы суставов берём по именам.
        self.qpos_indices = np.array([
            self.model.joint(name).qposadr[0] for name in JOINT_NAMES
        ])
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.home = command_pose(tuple(self.data.qpos[self.qpos_indices]))
        self.checked_pairs = self._load_pairs(model_dir / "t2_collision_pairs.csv")
        if not valid_pose(self.home) or not self.is_safe(self.home):
            raise RuntimeError("The Go1 home pose is unsafe")

    def _load_pairs(self, csv_path):
        pairs = set()
        checked_count = 0
        with csv_path.open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                if row["diagnostic_policy"] != "checked":
                    continue
                checked_count += 1
                ids = []
                for side in ("geom1", "geom2"):
                    geom_id = int(row[f"{side}_id"])
                    if not 0 <= geom_id < self.model.ngeom:
                        raise ValueError(f"Unknown Go1 geometry id: {geom_id}")
                    geom_name = self.model.geom(geom_id).name or f"UNNAMED_geom_{geom_id}"
                    if geom_name != row[side]:
                        raise ValueError(f"Go1 model differs from collision CSV at geom {geom_id}")
                    ids.append(geom_id)
                pairs.add(tuple(sorted(ids)))
        if checked_count != 655 or len(pairs) != 655:
            raise ValueError("The Go1 collision CSV must specify 655 unique checked pairs")
        return pairs

    def is_safe(self, pose):
        self.data.qpos[self.qpos_indices] = pose
        # нужны положения деталей и контакты. время и динамику модели не продвигаем.
        mujoco.mj_fwdPosition(self.model, self.data)
        contacts = self.data.contact
        deep_contacts = np.flatnonzero(contacts.dist < -MAX_PENETRATION_M)
        for index in deep_contacts:
            geom1, geom2 = contacts.geom[index]
            if tuple(sorted((int(geom1), int(geom2)))) in self.checked_pairs:
                return False
        return True

#!/usr/bin/env python3
"""Use the GIMP-aligned map while retaining the established stair exclusion."""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from generate_stair_keepout import build as build_stair_keepout


def build(root):
    root = Path(root)
    metadata, original, protected, *_ = build_stair_keepout(root / 'stair_keepout.yaml')
    aligned = np.array(Image.open(root / 'EVfloor9_gazebo_aligned.png').convert('L'))
    if aligned.shape != original.shape:
        raise ValueError('Aligned map must retain the original map dimensions')
    result = aligned.copy()
    result[original != protected] = 0
    with Image.open(root / 'EVfloor9_elevator_walls.png') as image:
        elevators = np.array(image.convert('RGBA'))
    if elevators.shape[:2] != aligned.shape:
        raise ValueError('Elevator wall layer must match the aligned map dimensions')
    elevator_walls = (elevators[:, :, 3] >= 128) & (elevators[:, :, :3].max(axis=2) < 90)
    result[elevator_walls] = 0
    metadata = dict(metadata, image='EVfloor9_formation_aligned.png')
    return metadata, aligned, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config')
    root = parser.parse_args().config_dir
    metadata, aligned, result = build(root)
    Image.fromarray(result).save(root / metadata['image'])
    (root / 'EVfloor9_formation_aligned.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False))
    print(f'Aligned formation map: {np.count_nonzero(result != aligned)} '
          'additional occupied cells from stair protection and elevator walls')


if __name__ == '__main__':
    main()

import pyneuroml.swc.ExportSWC as E
import logging

logger = logging.getLogger(__name__)

# Keep reference to the original
_orig_get = E._get_lines_for_seg_group

# Hotfix code to handle dangling trees (missing parents)
def _safe_get_lines_for_seg_group(cell, sg, type):
    try:
        return _orig_get(cell, sg, type)
    except KeyError as e:
        logger.warning(f"pyNeuroML SWC exporter missing parent {e!r}; defaulting to -1")
        # Re-run but monkey-patch the lookup dicts in-flight:
        # Temporarily backfill the missing key so the original logic continues
        missing_id = int(str(e))
        E.line_index_vs_distals[missing_id] = -1
        E.line_index_vs_proximals[missing_id] = -1
        return _orig_get(cell, sg, type)

# Apply the patch
E._get_lines_for_seg_group = _safe_get_lines_for_seg_group

import os, glob
# If you haven’t already:
#   pip install pyneuroml
from pyneuroml.swc.ExportSWC import convert_to_swc

input_folder  = 'c302_D_Full/cells'
output_folder = 'c302_D_Full/cells_swc'

for nml_file in glob.glob(os.path.join(input_folder, '*.cell.nml')):
    convert_to_swc(nml_file, add_comments=True, target_dir=output_folder)
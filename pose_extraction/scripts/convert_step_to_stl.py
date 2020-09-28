# freecad macro

from pathlib import Path

import ImportGui
import Mesh

step_root = Path('/home/rlha/d/iros-kit-template/STEP_Files')
stl_root = Path('/home/rlha/d/iros-kit-template/stl')

for step_file in step_root.glob('*.STEP'):
    if step_file.name == 'IROS2020_Practice.STEP':
        continue
    ImportGui.open(str(step_file))
    stl_name = step_file.name.replace('.STEP', '.stl')
    Mesh.export(App.ActiveDocument.Objects, str(stl_root / stl_name))
    App.closeDocument(App.ActiveDocument.Name)

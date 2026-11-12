"""Rebuild the scene using the bundled source model and geometry cache."""
import subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parent
for script in ['build_base.py','build.py']:
 subprocess.run([sys.executable,str(root/'sim'/script)],check=True)

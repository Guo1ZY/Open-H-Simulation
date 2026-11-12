"""Launch the bundled native MuJoCo viewer from any working directory."""
import runpy,sys
from pathlib import Path
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'sim'))
if len(sys.argv)==1:
 sys.argv += ['--mode','keyboard','--seconds','3600','--no-video']
runpy.run_path(str(root/'sim/native.py'),run_name='__main__')

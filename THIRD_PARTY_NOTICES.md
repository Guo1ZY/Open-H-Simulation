# Third-party models and notices

## C3VDv2 sigmoid model — CC BY 4.0

Original data: Golhar, Mayank V.; Galeano Fretes, Lucas Sebastian; Ayers, Loren; Akshintala, Venkata S.; Bobrow, Taylor L.; Durr, Nicholas J. (2025), **Data associated with the publication: C3VDv2: 3D colonoscopy video dataset with enhanced realism**, Johns Hopkins Research Data Repository, Version 3. DOI: https://doi.org/10.7281/T1/JC64MK .

- Project: https://durrlab.github.io/C3VDv2/
- Selected file: `c1_sigmoid1_model.zip`, containing `c1_sigmoid1.obj`.
- Official file: https://archive.data.jhu.edu/file.xhtml?fileId=14337&version=3.0
- License: **Creative Commons Attribution 4.0 International**, https://creativecommons.org/licenses/by/4.0/ . Legal code: https://creativecommons.org/licenses/by/4.0/legalcode .
- Original ZIP MD5 verified at download: `beeb4843927b1c3e00d0e9f81eb0f76a`.
- The extracted original OBJ is included under `assets/c3vdv2/source/`. The redundant source ZIP is not included. Detailed provenance is in `assets/c3vdv2/provenance.json`.

Modifications in `sim/colon/` and the scene: repaired winding, millimeter-to-meter conversion, rigid rotation and translation for inlet alignment, reduction to 45,000 visual triangles and 3,500 collision triangles, separate interior rendering, and approximately 2 mm outward extrusion into rigid collision prisms. A simple pink material is used; the package does not claim to include original endoscopic RGB textures. Attribution and license apply to the original model and these adaptations. Keep this notice when sharing. No endorsement by the original authors is implied.

Publication: Golhar et al., *C3VDv2—Colonoscopy 3D video dataset with enhanced realism*, Scientific Data (2026), https://doi.org/10.1038/s41597-026-07471-1 .

## UR5e robot model — BSD-3-Clause

The robot MJCF and meshes are the Universal Robots UR5e description from MuJoCo Menagerie, derived from the ROS-Industrial UR description. The robot is labeled UR5 in the project concept, but the bundled public geometry is UR5e.

- Model source: https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e
- Upstream description: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_e_description
- Copyright 2018 ROS Industrial Consortium.
- Full license: `ur5e/LICENSE` (**BSD-3-Clause**). Upstream README and changelog are retained in `ur5e/`.

Scene modifications duplicate the robot, change initial poses and collision masks, and add the motorized handle, compact gripper, flexible shaft and colon environment. Retain the upstream copyright, conditions and disclaimer when redistributing these assets.

## Python dependencies

Dependencies are installed separately through `requirements.txt` / `requirements-build.txt`; their source or binary distributions are not bundled in this ZIP. Their respective licenses continue to apply. This notice does not relicense third-party assets or imply a single license for the entire project.

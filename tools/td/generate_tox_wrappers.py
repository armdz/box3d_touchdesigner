"""
Generate Box3D wrapper TOX components inside TouchDesigner.

Run this script from a Text DAT in TD (Alt+R) or with run() from another DAT.
It creates 3 wrapper COMPs under /project1 and exports them to repo/tox/:

- Box3D_Solver.tox
- Box3D_Body.tox
- Box3D_Instances.tox

The wrappers contain built-in CPlusPlus operators preconfigured to load the
plugin DLLs from <repo>/plugin.
"""

from pathlib import Path


def _find_repo_root(start_dir):
	path = Path(start_dir).resolve()
	for candidate in [path] + list(path.parents):
		if (candidate / "CMakeLists.txt").exists() and (candidate / "plugin" / "Box3DCore.dll").exists():
			return candidate
	return None


def _set_first_existing_par(op_node, names, value):
	for name in names:
		par = getattr(op_node.par, name, None)
		if par is not None:
			par.val = value
			return name
	raise RuntimeError("Could not find parameter in {}: {}".format(op_node.path, names))


def _replace_comp(parent_comp, name):
	existing = parent_comp.op(name)
	if existing is not None:
		existing.destroy()
	return parent_comp.create(baseCOMP, name)


def _create_solver_wrapper(parent_comp, plugin_dir):
	comp = _replace_comp(parent_comp, "box3d_solver_wrapper")
	cpp = comp.create(cplusplusCHOP, "solver")
	dll = (plugin_dir / "Box3DSolverCHOP.dll").as_posix()
	_set_first_existing_par(cpp, ["plugin", "Plugin", "pluginpath", "Pluginpath"], dll)
	comp.comment = "Box3D Solver wrapper (world-only)."
	cpp.nodeX = 0
	cpp.nodeY = 0
	return comp


def _create_body_wrapper(parent_comp, plugin_dir):
	comp = _replace_comp(parent_comp, "box3d_body_wrapper")
	cpp = comp.create(cplusplusSOP, "body")
	dll = (plugin_dir / "Box3DBodySOP.dll").as_posix()
	_set_first_existing_par(cpp, ["plugin", "Plugin", "pluginpath", "Pluginpath"], dll)
	comp.comment = "Box3D Body wrapper (single rigid body SOP)."
	cpp.nodeX = 0
	cpp.nodeY = 0
	return comp


def _create_instances_wrapper(parent_comp, plugin_dir):
	comp = _replace_comp(parent_comp, "box3d_instances_wrapper")
	cpp = comp.create(cplusplusCHOP, "instances")
	dll = (plugin_dir / "Box3DBodiesCHOP.dll").as_posix()
	_set_first_existing_par(cpp, ["plugin", "Plugin", "pluginpath", "Pluginpath"], dll)
	comp.comment = "Box3D Instances wrapper (multi-body CHOP)."
	cpp.nodeX = 0
	cpp.nodeY = 0
	return comp


def _save_tox(comp, output_path):
	output_path.parent.mkdir(parents=True, exist_ok=True)
	comp.save(output_path.as_posix())


def main():
	repo = _find_repo_root(project.folder)
	if repo is None:
		raise RuntimeError(
			"Could not locate repo root from project.folder='{}'. Open a .toe inside the repo first.".format(project.folder)
		)

	plugin_dir = repo / "plugin"
	if not (plugin_dir / "Box3DSolverCHOP.dll").exists():
		raise RuntimeError("Missing plugin DLLs in {}. Build first (cmake --build build --config Release).".format(plugin_dir))

	root_comp = op("/project1")
	if root_comp is None:
		raise RuntimeError("Could not find /project1")

	solver = _create_solver_wrapper(root_comp, plugin_dir)
	body = _create_body_wrapper(root_comp, plugin_dir)
	instances = _create_instances_wrapper(root_comp, plugin_dir)

	# Layout in the network editor
	solver.nodeX, solver.nodeY = -500, 200
	body.nodeX, body.nodeY = -200, 200
	instances.nodeX, instances.nodeY = 100, 200

	tox_dir = repo / "tox"
	_save_tox(solver, tox_dir / "Box3D_Solver.tox")
	_save_tox(body, tox_dir / "Box3D_Body.tox")
	_save_tox(instances, tox_dir / "Box3D_Instances.tox")

	print("Saved TOX wrappers to {}".format(tox_dir.as_posix()))


if __name__ == "__main__":
	main()

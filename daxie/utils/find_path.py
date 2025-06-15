from importlib.resources import files


def get_so100_path():
    urdf_path = str(
        files("daxie").joinpath("robots/so-100/SO_5DOF_ARM100_8j/so100.urdf")
    )
    meshes_path = str(
        files("daxie").joinpath("robots/so-100/SO_5DOF_ARM100_8j/meshes/")
    )

    return urdf_path, meshes_path

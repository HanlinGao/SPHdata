import open3d as o3d
import numpy as np
import tempfile
import os
import subprocess
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


PARTICLE_RADIUS = 0.025
VOLUME_SAMPLING_BIN = '../../SPlisHSPlasH/bin/VolumeSampling'


def numpy_from_bgeo(path):
    import partio
    p = partio.read(path)
    print(path)
    print(p)
    pos = p.attributeInfo('position')
    vel = p.attributeInfo('velocity')
    ida = p.attributeInfo('trackid')  # old format
    if ida is None:
        ida = p.attributeInfo('id')  # new format after splishsplash update
    n = p.numParticles()
    pos_arr = np.empty((n, pos.count))
    for i in range(n):
        pos_arr[i] = p.get(pos, i)

    vel_arr = None
    if not vel is None:
        vel_arr = np.empty((n, vel.count))
        for i in range(n):
            vel_arr[i] = p.get(vel, i)

    if not ida is None:
        id_arr = np.empty((n,), dtype=np.int64)
        for i in range(n):
            id_arr[i] = p.get(ida, i)[0]

        s = np.argsort(id_arr)
        result = [pos_arr[s]]
        if not vel is None:
            result.append(vel_arr[s])
    else:
        result = [pos_arr, vel_arr]

    return tuple(result)


def obj_surface_to_particles(objpath, radius=None):
    if radius is None:
        radius = PARTICLE_RADIUS
    obj = o3d.io.read_triangle_mesh(objpath)
    particle_area = np.pi * radius**2
    # 1.9 to roughly match the number of points of SPlisHSPlasHs surface sampling
    num_points = int(1.9 * obj.get_surface_area() / particle_area)
    pcd = obj.sample_points_poisson_disk(num_points, use_triangle_normal=True)
    points = np.asarray(pcd.points).astype(np.float32)
    normals = -np.asarray(pcd.normals).astype(np.float32)
    return points, normals


def obj_volume_to_particles(objpath, scale=1, radius=None):
    if radius is None:
        radius = PARTICLE_RADIUS
    with tempfile.TemporaryDirectory() as tmpdir:
        outpath = os.path.join(tmpdir, 'out.bgeo')
        scale_str = '{0}'.format(scale)
        radius_str = str(radius)
        status = subprocess.run([
            VOLUME_SAMPLING_BIN, '-i', objpath, '-o', outpath, '-r', radius_str,
            '-s', scale_str
        ])
        return numpy_from_bgeo(outpath)


def write_particles(path_without_ext, pos, vel=None):
    """Writes the particles as point cloud ply.
    Optionally writes particles as bgeo which also supports velocities.
    """
    arrs = {'pos': pos}
    if not vel is None:
        arrs['vel'] = vel
    np.savez('../out_ply/' + path_without_ext + '.npz', **arrs)

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pos))
    o3d.io.write_point_cloud('../out_ply/' + path_without_ext + '.ply', pcd)


def prepare_particles(scene, output_dir):
    """generate initial data of box, box_normals, fluid_pos, fluid_vel according to the input json file

    :param scene: set path to box.obj and fluid.obj, as well as the translation of positions
    :return: numpy arrays of box, box_normals, fluid_pos, fluid_vel
    """
    # prepare static particles
    walls = []
    for x in scene['walls']:
        points, normals = obj_surface_to_particles(x['path'])
        if 'invert_normals' in x and x['invert_normals']:
            normals = -normals
        points += np.asarray([x['translation']], dtype=np.float32)
        walls.append((points, normals))
    box = np.concatenate([x[0] for x in walls], axis=0)
    box_normals = np.concatenate([x[1] for x in walls], axis=0)
    
    # export static particles
    write_particles(os.path.join(output_dir, 'box'), box, box_normals)

    # prepare fluids
    fluids = []
    for x in scene['fluids']:
        points = obj_volume_to_particles(x['path'])[0]
        points += np.asarray([x['translation']], dtype=np.float32)
        velocities = np.empty_like(points)
        velocities[:, 0] = x['velocity'][0]
        velocities[:, 1] = x['velocity'][1]
        velocities[:, 2] = x['velocity'][2]
        range_ = range(x['start'], x['stop'], x['step'])
        fluids.append(
            (points.astype(np.float32), velocities.astype(np.float32), range_))

    pos = np.empty(shape=(0, 3), dtype=np.float32)
    vel = np.empty_like(pos)
    for points, velocities, range_ in fluids:
        pos = np.concatenate([pos, points], axis=0)
        vel = np.concatenate([vel, velocities], axis=0)
    
    # export fluid particles
    write_particles(os.path.join(output_dir, 'fluids'), pos, vel)

    return box, box_normals, pos, vel


def mat_visualize_prepared(prepared_box=np.empty(shape=(0, 3)), prepared_fluids=np.empty(shape=(0, 3))):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    if len(prepared_box):
        box_x = prepared_box[:, 0]
        box_y = prepared_box[:, 1]
        box_z = prepared_box[:, 2]
        ax.scatter(box_x, box_z, box_y, c='whitesmoke', marker='o', alpha=0.1)
    if len(prepared_fluids):
        fluid_x = prepared_fluids[:, 0]
        fluid_y = prepared_fluids[:, 1]
        fluid_z = prepared_fluids[:, 2]
        ax.scatter(fluid_x, fluid_z, fluid_y, c='black', marker='o')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 4)


    plt.show()
# def o3d_visualize_prepared(path_to_pcd):
#     # render box
#     box_pcd = o3d.io.read_point_cloud(os.path.join(path_to_pcd, 'box.ply'))
#     fluid_pcd = o3d.io.read_point_cloud(os.path.join(path_to_pcd, 'fluids.ply'))
#     o3d.visualization.draw_geometries([box_pcd, fluid_pcd])


if __name__ == '__main__':
    scene = '../settings/example_scene.json'
    with open(scene, 'r') as f:
        scene_obj = json.load(f)
    box, box_normals, pos, vel = prepare_particles(scene_obj, 'out_put')
    print(np.max(box, axis=0))
    print(np.min(box, axis=0))

    print(np.max(pos, axis=0))
    print(np.min(pos, axis=0))
    mat_visualize_prepared(prepared_fluids=pos)

    # o3d_visualize_prepared('../out_ply/out_put')
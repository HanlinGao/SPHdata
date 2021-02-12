import os
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation


def draw_animation_plot(result_dir):
    # get pos data in the form of [[x1, x2, ...], [y1, y2, ...], [z1, z2, ...]]
    all_timesteps = get_timestep_data(result_dir)

    x = all_timesteps[0][0]
    y = all_timesteps[0][1]
    z = all_timesteps[0][2]

    fig = plt.figure(figsize=(10, 10))
    ax = p3.Axes3D(fig)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d([-1, 1])
    ax.set_ylim3d([-1, 1])
    ax.set_zlim3d([0, 4])

    points, = ax.plot(x, y, z, 'o', alpha=0.4)
    txt = fig.suptitle('')

    def update_points(frame, x, y, z, points):
        txt.set_text('timestep={:d}'.format(frame))
        new_x = all_timesteps[frame][0]
        new_y = all_timesteps[frame][1]
        new_z = all_timesteps[frame][2]

        points.set_data(new_x, new_y)
        points.set_3d_properties(new_z, 'z')

        return points, txt
    ani = animation.FuncAnimation(fig, update_points, frames=len(all_timesteps), fargs=(x, y, z, points))
    plt.show()


def get_timestep_data(result_dir):
    """
    generate a list of arrays of positions of animated points

    Args:
        result_dir : directory consisting of .npz and .ply files
    
    Returns:
        fluid: The list of arrays of positions of animated points
    """
    path = os.path.join('../result', result_dir)
    files = os.listdir(path)
    files = list(filter(lambda f: '.npz' in f and 'fluid' in f, files))
    files.sort()
    
    fluid = []
    for f in files:
        data = np.load(os.path.join(path, f))['pos']
        # the y value in data is actually the z-value in 3D visualization
        fluid.append([data[:, 0], data[:, 2], data[:, 1]])
    return fluid


def draw_animation_scatter(result_dir):
    # get pos data in the form of [[x1, x2, ...], [y1, y2, ...], [z1, z2, ...]]
    all_timesteps = get_timestep_data(result_dir)

    x = all_timesteps[0][0]
    y = all_timesteps[0][1]
    z = all_timesteps[0][2]

    def update_graph(frame):
        title.set_text('timestep={:d}'.format(frame))
        new_x = all_timesteps[frame][0]
        new_y = all_timesteps[frame][1]
        new_z = all_timesteps[frame][2]
        graph._offsets3d = (new_x, new_y, new_z)

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    title = ax.set_title('')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d([-1, 1])
    ax.set_ylim3d([-1, 1])
    ax.set_zlim3d([0, 4])
    ax.view_init(elev=10, azim=30)

    graph = ax.scatter(x, y, z, 'o', alpha=0.2, edgecolors='k')

    ani = animation.FuncAnimation(fig, update_graph, frames=len(all_timesteps))
    plt.show()


if __name__ == '__main__':
    draw_animation_scatter('upper_out')
    # draw_animation('bottom_out')
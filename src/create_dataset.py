import os
import json
import pysplishsplash as sph
import shutil
import pyvista as pv
import numpy as np
import pickle
import re


def create_dateset(template, setting, dataset):
    """modify template with setting, create training data based on modified template

    :param template: template json file in the [template] directory
    :param setting: setting json file in the [settings] directory
    :param dataset: dataset directory that will be created
    """
    modify_template(template, setting, dataset)
    run_simulator(dataset)


def create_clean_dir(dirname):
    if not os.path.exists(dirname):
        os.mkdir(dirname)
        print(f'Creating directory: {dirname}')
    else:
        print(f'Clean up the directory: {dirname}')
        for root, dirs, files in os.walk(dirname):
            for file in files:
                os.remove(os.path.join(root, file))
            for dir in dirs:
                shutil.rmtree(os.path.join(root, dir))


def modify_template(template, setting, modified_dir):
    """modify template with setting, generate modified json files that are used in simulator

    :param template: a json file provided in SPlishSPlash/data/Scenes
    :param setting: a json file containing initial settings
    [
        {
        "start": [ , , ],
        "end": [ , , ],
        "initialVelocity": [, , ]
        }
    ]
    :param modified_dir: each block in the setting corresponds to one json file, thus multiple of them
    """
    root = os.path.dirname(os.getcwd())
    template_file = os.path.join(root, 'template', template)
    setting_file = os.path.join(root, 'settings', setting)

    modified_dir_path = os.path.join(root, 'intermediate', modified_dir, 'Jsons')
    create_clean_dir(modified_dir_path)

    with open(template_file, 'r') as ft:
        template_content = json.load(ft)
    with open(setting_file, 'r') as fs:
        setting_content = json.load(fs)

    length = len(setting_content)
    for i in range(length):
        template_content['FluidBlocks'][0]['start'] = setting_content[i]['start']
        template_content['FluidBlocks'][0]['end'] = setting_content[i]['end']
        template_content['FluidBlocks'][0]['initialVelocity'] = setting_content[i]['initialVelocity']

        save_json = str(i) + '.json'
        with open(os.path.join(modified_dir_path, save_json), 'w') as fw:
            json.dump(template_content, fw)


def run_simulator(modified_dir):
    """run simulator with each json_file as setting, output corresponding pickle file

    :param modified_dir: directory of modified json files
    """
    root = os.path.dirname(os.getcwd())
    json_dir = os.path.join(root, 'intermediate', modified_dir, 'Jsons')
    output_dir = os.path.join(root, 'intermediate', modified_dir, 'VTK')
    create_clean_dir(output_dir)

    json_files = os.listdir(json_dir)
    pickle_dir = os.path.join(root, 'datasets', modified_dir)
    create_clean_dir(pickle_dir)
    for scene in json_files:
        scene_file = os.path.join(json_dir, scene)
        vtk_file = os.path.join(output_dir, os.path.splitext(scene)[0])
        base = sph.Exec.SimulatorBase()
        base.init(sceneFile=scene_file, useGui=True, initialPause=True,
                  useCache=True, stopAt=-1.0, stateFile='', outputDir=vtk_file,
                  param='')

        gui = sph.GUI.Simulator_GUI_imgui(base)
        base.setGui(gui)
        base.run()

        pickle_file = os.path.join(pickle_dir, os.path.splitext(scene)[0] + '.pkl')
        generate_dataset(vtk_file, pickle_file)

    # clean up the vtk files to save space as do not need them later
    create_clean_dir(output_dir)


def generate_dataset(vtk_dir, output_file):
    """read vtk files generated by the simulator, generate a pickle file containing data in the form of
    [t1, t2, t3, ...., tn] where t_i = [pos_matrix, vel_matrix, pos_matrix_label1, pos_matrix_label2]

    :param vtk_dir: directory of vtk files generated by simulator
    :param output_file: output pickle file
    """
    dataset = []
    root_path = os.path.dirname(os.getcwd())
    vtk_dir = os.path.join(root_path, 'intermediate', vtk_dir, 'vtk')

    regex = re.compile(r"\d+")
    for root, dirs, files in os.walk(vtk_dir):
        files.sort(key=lambda x: int(max(regex.findall(x))))
        for name in files:
            mesh = pv.read(os.path.join(vtk_dir, name))
            pos_matrix = np.array(mesh.points)
            vel_matrix = np.array(mesh.point_arrays['velocity'])
            ids = np.array(mesh.point_arrays['id'])

            # particle is stored in different order in different time, so need ordering
            pos_matrix = np.insert(pos_matrix, 0, values=ids, axis=1)
            vel_matrix = np.insert(vel_matrix, 0, values=ids, axis=1)
            # print(pos_matrix)
            pos_matrix = pos_matrix[np.argsort(pos_matrix[:, 0])]
            vel_matrix = vel_matrix[np.argsort(vel_matrix[:, 0])]

            # print(pos_matrix)
            dataset.append([pos_matrix[:, 1:], vel_matrix[:, 1:]])

    # print(len(dataset))
    # add label, finally the dataset will be [pos_matrix, vel_matrix, pos_matrix_t+1, pos_matrix_t+2]
    for j in range(len(dataset) - 2):
        dataset[j].append(dataset[j + 1][0][:])
        dataset[j].append(dataset[j + 2][0][:])

    # save into a .pkl file
    if os.path.exists(output_file):
        print('clean up', output_file)
        os.remove(output_file)
    with open(output_file, 'wb') as f:
        pickle.dump(dataset[: -2], f)

    print(len(dataset))


if __name__ == '__main__':
    create_dateset('Sampling_2D.json', 'fill_data.json', 'fill_data')


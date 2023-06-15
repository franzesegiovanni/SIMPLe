# %%
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import os 

data = {
    'Trajectory': [],
    'Error': []
}
error_complete=[]
for trial in range(5):
    data['Trajectory'].append(trial+1)
    directory = str(pathlib.Path().resolve()) + '/Box_correction_' + str(trial+1) + '/'

    files = os.listdir(directory)
    files.sort()
    print(files)
    loaded_files = []
    for file_name in files:
        file_path = os.path.join(directory, file_name)
        loaded_file = np.load(file_path)
        loaded_files.append(loaded_file)

    recording_freq = 20
    fig, ax = plt.subplots(3, 1, sharex='col', sharey='row')
    error=[]
    for index, trajectory in enumerate(loaded_files[-5:]):
        time= np.linspace(0, trajectory['recorded_cart_pose'].shape[1],trajectory['recorded_cart_pose'].shape[1])/recording_freq
        for i in range (3):
            ax[i].plot(time, trajectory['recorded_tag_position'][i])
        error.append(np.linalg.norm(loaded_files[-6]['recorded_tag_position'][:,-1]-trajectory['recorded_tag_position'][:,-1]))
    mean=np.mean(error)
    std=np.std(error)
    error.append(mean)
    error.append(std)       
    error_complete.append(error)        
    ax[0].axhline(y=loaded_files[-6]['recorded_tag_position'][0][-1], color='red', linestyle='--')
    ax[1].axhline(y=loaded_files[-6]['recorded_tag_position'][1][-1], color='red', linestyle='--')
    ax[2].axhline(y=loaded_files[-6]['recorded_tag_position'][2][-1], color='red', linestyle='--')    
    ax[0].set_ylabel('X')
    ax[1].set_ylabel('Y')
    ax[2].set_ylabel('Z')


fig, ax = plt.subplots()
ax.axis('off')  # Hide axes

plt.title("Error in execution")
# Add an extra row and column labels
row_labels = ['Trial 1', 'Trial 2', 'Trial 3', 'Trial 4', 'Trial 5']
col_labels = ['Execution 1', 'Execution 2', 'Execution 3', 'Execution 4', 'Execution 5', 'Mean', 'Std']

table = plt.table(cellText=error_complete,
                  loc='center',
                  cellLoc='center', 
                  rowLabels=row_labels,
                  colLabels=col_labels)

table.auto_set_font_size(False)
table.set_fontsize(12)
table.scale(1.2, 1.2)

for trial in range(5):
    data['Trajectory'].append(trial+1)
    directory = str(pathlib.Path().resolve()) + '/Box_correction_' + str(trial+1) + '/'

    files = os.listdir(directory)
    files.sort()
    print(files)
    loaded_files = []
    for file_name in files:
        file_path = os.path.join(directory, file_name)
        loaded_file = np.load(file_path)
        loaded_files.append(loaded_file)

    recording_freq = 20
    fig, ax = plt.subplots(2, 1, sharex='col', sharey='row')
    error=[]
    for index, trajectory in enumerate(loaded_files[:-5]):
        time= np.linspace(0, trajectory['recorded_force_torque'].shape[1],trajectory['recorded_force_torque'].shape[1])/recording_freq
        ax[0].plot(time, np.linalg.norm(trajectory['recorded_force_torque'][:6], axis=0), label=str(index))
        ax[1].plot(time, np.linalg.norm(trajectory['recorded_force_torque'][6:], axis=0), label=str(index))
        ax[0].legend()
        ax[1].legend()
plt.show()

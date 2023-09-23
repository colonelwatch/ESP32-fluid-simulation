import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

with open('sim_params.json', 'r') as f:
    sim_params = json.load(f)

N = sim_params['N']
DT = sim_params['DT']
SECONDS = sim_params['SECONDS']
FRAMERATE = sim_params['OUTPUT_FPS']

def read_field_file(file_path, type):
    if type != 'scalar' and type != 'vector':
        raise ValueError('type must be scalar or vector')

    file_str = open(file_path, 'r').read()
    frame_arr = file_str.split('\n\n')
    frame_arr = [frame for frame in frame_arr if frame]
    frame_arr = [frame.split('\n') for frame in frame_arr]
    frame_arr = [[row.split(' ') for row in frame] for frame in frame_arr]

    if type == 'scalar':
        frame_arr = [[[float(item) for item in row] for row in frame] for frame in frame_arr]
    elif type == 'vector':
        def string_to_vector(string):
            string = string.replace('(', '')
            string = string.replace(')', '')
            pair = tuple(string.split(','))
            pair = (float(pair[0]), float(pair[1]))
            return pair
        frame_arr = [[[string_to_vector(item) for item in row] for row in frame] for frame in frame_arr]

    frame_arr = np.array(frame_arr)

    return frame_arr

velocity_frames = read_field_file('sim_velocity.txt', 'vector')
color_frames = read_field_file('sim_color.txt', 'scalar')

# see ESP32-fluid-simulation for how and why percent density error is constructed
divergence_frames = np.abs(read_field_file('sim_divergence.txt', 'scalar'))
pct_rho_err_frames = np.array([100*DT*divergence_frame for divergence_frame in divergence_frames])

frame_interval = 1000//FRAMERATE
frame_count = velocity_frames.shape[0]

fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
ax1.set_title('Velocity')
ax2.set_title('Color')
ax3.set_title('Pct density error')

# plot velocity field as if we used imshow
ax1.set_aspect('equal')
ax1.invert_yaxis()

artists = []
foo = np.random.random(size=(N, N))
artists.append(ax1.quiver(foo, foo, scale=100, scale_units='inches', color='blue'))
artists.append(ax2.imshow(foo, interpolation='nearest', vmin=0, vmax=1, animated=True))
artists.append(ax3.imshow(foo, cmap='hot', interpolation='nearest', vmin=0, vmax=100, animated=True))

def update(i):
    # convert from i-j to ordinary Cartesian (x-y)
    u = velocity_frames[i, :, :, 1]
    v = -velocity_frames[i, :, :, 0]

    color_frame = color_frames[i, :, :]
    pct_rho_err_frame = pct_rho_err_frames[i, :, :]

    artists[0].set_UVC(u, v)
    artists[1].set_array(color_frame)
    artists[2].set_array(pct_rho_err_frame)

    return artists

ani = animation.FuncAnimation(fig, update, frames=frame_count, interval=frame_interval, blit=True)
plt.show()
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

DT = 0.01
FRAMERATE = 60
N_ROWS = 64
SECONDS = 10

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

def read_velocity():
    return read_field_file('velocity.txt', 'vector')
def read_pressure():
    return read_field_file('pressure.txt', 'scalar')
def read_divergence(absolute = True):
    divergence = read_field_file('divergence.txt', 'scalar')
    if(absolute):
        divergence = np.abs(divergence)
    return divergence
def read_color():
    return read_field_file('color.txt', 'scalar')

velocity_frames = read_velocity()
pressure_frames = read_pressure()
color_frames = read_color()
divergence_frames = read_divergence()

frame_interval = 1000//FRAMERATE
frame_count = velocity_frames.shape[0]

fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
ax1.set_title('Pressure and Velocity')
ax2.set_title('Color')
ax3.set_title('Absolute Divergence (Bad!)')

artists = []
foo = np.random.random(size=(64, 64))
artists.append(ax1.quiver(foo, foo, scale=100, scale_units='inches', color='blue'))
artists.append(ax1.imshow(foo, cmap='hot', interpolation='nearest', vmin=-2, vmax=2, animated=True))
artists.append(ax2.imshow(foo, interpolation='nearest', vmin=0, vmax=1, animated=True))
artists.append(ax3.imshow(foo, cmap='hot', interpolation='nearest', vmin=0, vmax=1, animated=True))

def update(i):
    u = velocity_frames[i, :, :, 0]
    v = velocity_frames[i, :, :, 1]
    pressure_frame = pressure_frames[i, :, :]
    color_frame = color_frames[i, :, :]
    divergence_frame = divergence_frames[i, :, :]

    artists[0].set_UVC(u, v)
    artists[1].set_array(pressure_frame)
    artists[2].set_array(color_frame)
    artists[3].set_array(divergence_frame)

    return artists

ani = animation.FuncAnimation(fig, update, frames=frame_count, interval=frame_interval, blit=True)
plt.show()
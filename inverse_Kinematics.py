from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import numpy as np

base_coord = np.array([0., 0., 50.])

lengths = {
    "lower": 250,
    "upper": 250,
    "tool": 100,
}

home_angles = {
    "base": 90,
    "upper": 90,
    "lower": 90,
    "tool_x": 90,
    "tool_y": 90,
}

lower_vec = np.array([0., 0., lengths['lower']])
upper_vec = np.array([0., lengths['upper'], 0.])
tool_vec = np.array([0., lengths['tool'], 0.])

wx, wy, wz = 500, 500, 1000  # working space


def rad(dg):
    return np.radians(dg)


def deg(rd):
    return np.rad2deg(rd)


def normalize(vector):
    return vector / np.linalg.norm(vector)


def length(vector):
    x, y, z = vector
    return np.sqrt(x**2 + y**2 + z**2)


def angle_between(vector1, vector2):
    return deg(np.arccos(np.dot(normalize(vector1), normalize(vector2))))


def rotation(vector, axis, theta):
    theta = rad(theta)
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.dot(vector, np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                                    [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                                    [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]]))


def solve(target, x_angle=0, z_angle=0):
    # print("target: [{0}, {1}, {2}] angles: x: {3}, y: {4}\n".format(*target, x_angle, z_angle))
    z_angle += 180
    tool_vec_inv = tool_vec
    tool_vec_inv = rotation(tool_vec_inv, [0, 0, 1], z_angle)
    tool_vec_inv = rotation(tool_vec_inv, [1, 0, 0], x_angle)
    upper_point_inv = target + tool_vec_inv
    base_rotation = [upper_point_inv[0], upper_point_inv[1], base_coord[2]] - base_coord
    base_rotation = deg(np.arctan(base_rotation[0]/base_rotation[1]))
    arm_joints_axis = rotation([1, 0, 0], [0, 0, 1], base_rotation)
    arm_vector = upper_point_inv - base_coord
    arm_l = length(arm_vector)
    base_angle = deg(np.arccos((arm_l**2 + lengths['lower']**2 - lengths['upper']**2)/(2.0*arm_l*lengths['lower'])))
    arm_angle = angle_between(arm_vector, np.array([arm_vector[0], arm_vector[1], 0]))
    base_angle += arm_angle - 90
    base_angle *= -1
    lower_vec_inv = rotation(lower_vec, arm_joints_axis, base_angle)
    lower_point_inv = base_coord + lower_vec_inv
    lower_angle = deg(np.arccos((lengths['upper']**2+lengths['lower']**2-arm_l**2)/(2.0*lengths['upper']*lengths['lower'])))
    upper_vec_inv = upper_point_inv-lower_point_inv
    tool_x_angle = angle_between(upper_vec_inv, np.array([upper_vec_inv[0], upper_vec_inv[1], 0])) - x_angle
    tool_z_angle = angle_between(np.array([tool_vec_inv[0], tool_vec_inv[1], 0]),
                                 np.array([upper_vec_inv[0], upper_vec_inv[1], 0]))

    '''
    print('lengths for verifying')
    print(length(lower_vec_inv))
    print(length(upper_point_inv-lower_point_inv))
    print(length(tool_vec_inv))
    '''
    if round(length(upper_point_inv-lower_point_inv)) != lengths['upper']:
        print('CALCULATION ERROR')

    base_angle += 90
    base_rotation += 90
    tool_x_angle += 90
    tool_z_angle -= 90

    '''
    print("\nbase rotation: %d" % base_rotation)
    print("base angle: %d" % base_angle)
    print("lower angle: %d" % lower_angle)
    print("upper angle: %d" % tool_x_angle)
    print("tool rotation: %d" % tool_z_angle)
    '''

    ik_points = [target, upper_point_inv, lower_point_inv, base_coord, [0, 0, 0]]

    ik_angles = [z_angle, x_angle, base_rotation, base_angle, lower_angle, tool_x_angle, tool_z_angle]

    return ik_points, ik_angles


def main():
    def update_line(hl, xdata, ydata, zdata):
        hl.set_xdata(np.asarray(xdata))
        hl.set_ydata(np.asarray(ydata))
        hl.set_3d_properties(np.asarray(zdata))
        plt.draw()

    def update_dots(sc, xdata, ydata, zdata):
        sc.set_xdata(np.asarray(xdata))
        sc.set_ydata(np.asarray(ydata))
        sc.set_3d_properties(np.asarray(zdata))
        plt.draw()

    plt.style.use('dark_background')
    map = plt.figure()
    map_ax = Axes3D(map)
    map_ax.autoscale(enable=True, axis='both', tight=True)

    map_ax.set_xlim3d([-500.0, 500.0])
    map_ax.set_ylim3d([-500.0, 500.0])
    map_ax.set_zlim3d([0.0, 500.0])
    hl, = map_ax.plot3D([0], [0], [0], c='white', linewidth=6)
    # sc, = map_ax.scatter3D([0], [0], [0], c='red', s=100)

    for i in range(10000):

        # target = [np.cos(i/10)*200, 390, np.abs(np.sin(i/10)*100)]
        target = [np.cos(i / 10) * 100, np.sin(i / 10) * 100+300, 200]
        # target = [0, 200, 200]
        # points, _ = solve(target, np.cos(i/20)*30, np.sin(i/20)*30)
        points, _ = solve(target,  0, np.cos(i / 10) * 45)
        # corners = [[-wx,-wy,0],[wx,-wy,0],[-wx,wy,0],[wx,wy,0],[-wx,-wy,wz],[wx,-wy,wz],[-wx,wy,wz],[wx,wy,wz]]
        points_for_plotting = [[p[0] for p in points], [p[1] for p in points], [p[2] for p in points]]
        # corners_for_plotting = [[p[0] for p in corners], [p[1] for p in corners], [p[2] for p in corners]]

        # ax.scatter3D(*points_for_plotting, c='black', s=100)
        # ax.plot3D(*points_for_plotting, c='white', linewidth=6)
        # ax.scatter3D(*corners_for_plotting, c='yellow')
        # ax.scatter3D(*target, c='red', s=150)
        update_line(hl, *points_for_plotting)
        # update_dots(sc, *points_for_plotting)
        plt.show(block=False)
        plt.pause(0.02)

    return 0


if __name__ == '__main__':
    main()

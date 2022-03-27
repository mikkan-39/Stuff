import matplotlib.pyplot as plt
# from numpy import *
import numpy as np
from time import time

start = time()
width, height = 125, 90
camera = np.array([0, -10, 0])

light = {'position': np.array([-0.5, 0, 0.5]),
         'ambient': np.array([1, 1, 1]),
         'diffuse': np.array([1, 1, 1]),
         'specular': np.array([1, 1, 1])}

spheres = [
    {'center': np.array([0, 1.8, 0]), 'radius': 0.5, 'ambient': np.array([0, 0, 0.1]),
     'diffuse': np.array([0, 0, 0.7]), 'specular': np.array([0.2, 0.2, 0.3]), 'shininess': 100},

    {'center': np.array([0.35, 0.7, 0.35]), 'radius': 0.1, 'ambient': np.array([0.1, 0, 0]),
     'diffuse': np.array([0.7, 0, 0]), 'specular': np.array([1, 1, 1]), 'shininess': 100},

    {'center': np.array([-0.4, 1, -0.2]), 'radius': 0.15, 'ambient': np.array([0, 0.1, 0]),
     'diffuse': np.array([0, 0.6, 0]), 'specular': np.array([0.1, 0.1, 0.1]), 'shininess': 10},



    {'center': np.array([0, 0, -9000]), 'radius': 9000 - 0.5, 'ambient': np.array([0.1, 0.1, 0.1]),
     'diffuse': np.array([0.6, 0.6, 0.6]), 'specular': np.array([0.1, 0.1, 0.1]), 'shininess': 10},

    {'center': np.array([0, 9000, 0]), 'radius': 9000 - 4, 'ambient': np.array([0.1, 0.02, 0.1]),
     'diffuse': np.array([0.6, 0.12, 0.6]), 'specular': np.array([0.1, 0.1, 0.1]), 'shininess': 10},

    {'center': np.array([-9000, 0, 0]), 'radius': 9000 - 0.8, 'ambient': np.array([0.1, 0.1, 0.02]),
     'diffuse': np.array([0.6, 0.6, 0.12]), 'specular': np.array([0.1, 0.1, 0.1]), 'shininess': 10},

    {'center': np.array([9000, 0, 0]), 'radius': 9000 - 0.8, 'ambient': np.array([0.02, 0.1, 0.1]),
     'diffuse': np.array([0.12, 0.6, 0.6]), 'specular': np.array([0.1, 0.1, 0.1]), 'shininess': 10},

]


def normalize(vector):
    return vector / np.linalg.norm(vector)


def sphere_intersect(center, radius, ray_origin, ray_direction):
    b = 2 * np.dot(ray_direction, ray_origin - center)
    c = np.linalg.norm(ray_origin - center) ** 2 - radius ** 2
    delta = b ** 2 - 4 * c
    if delta > 0:
        t1 = (-b + np.sqrt(delta)) / 2
        t2 = (-b - np.sqrt(delta)) / 2
        if t1 > 0 and t2 > 0:
            return min(t1, t2)
    return None


def nearest_intersected_object(ray_origin, ray_direction):
    distances = [sphere_intersect(s['center'], s['radius'], ray_origin, ray_direction) for s in spheres]
    objects = spheres
    nearest_object = None
    min_distance = np.inf
    for index, distance in enumerate(distances):
        if distance and distance < min_distance:
            min_distance = distance
            nearest_object = objects[index]
    return nearest_object, min_distance


def draw_ray(origin, direction):
    nearest_object, distance = nearest_intersected_object(origin, direction)
    if nearest_object:
        intersection_point = origin + direction * distance
        normal_to_surface = normalize(intersection_point - nearest_object['center'])
        shifted_point = intersection_point + 1e-5 * normal_to_surface
        intersection_to_light_direction = normalize(light['position'] - shifted_point)
        intersection_to_light_distance = np.linalg.norm(light['position'] - shifted_point)
        _, min_distance = nearest_intersected_object(shifted_point, intersection_to_light_direction)
        is_shadowed = min_distance < intersection_to_light_distance
        intersection_to_camera_direction = normalize(camera - intersection_point)
        H = normalize(intersection_to_light_direction + intersection_to_camera_direction)
        illumination = np.zeros((3))
        illumination += nearest_object['ambient'] * light['ambient'] * np.dot(normal_to_surface, intersection_to_light_direction)
        if not is_shadowed:
            illumination += nearest_object['diffuse'] * light['diffuse'] * np.dot(normal_to_surface,intersection_to_light_direction)
            illumination += nearest_object['specular'] * light['specular'] * np.dot(normal_to_surface, H) ** (nearest_object['shininess'] / 4)
        return np.clip(illumination, 0, 1)
    return 0


def main():
    ratio = float(width) / float(height)
    screen = (-1., 1. / ratio, 1., -1. / ratio)
    fig, ax = plt.subplots(1, 1)
    image = np.zeros((height, width, 3))
    im = ax.imshow(image)
    plt.pause(0.1)
    for i, z in enumerate(np.linspace(screen[1], screen[3], height)):
        for j, x in enumerate(np.linspace(screen[0], screen[2], width)):
            pixel = np.array([x, 1, z])
            origin = camera
            direction = normalize(pixel - origin)

            image[i, j] = draw_ray(origin, direction)
        print("progress: %d" % ((i + 1)/height*100)+"%")
        im.set_data(image)
        fig.canvas.draw_idle()
        plt.pause(0.01)
    plt.imsave('0.png', image)
    print(str(round(time()-start))+'s')
    return 0


if __name__ == '__main__':
    main()

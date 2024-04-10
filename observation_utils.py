import numpy as np

from map_utils import *

from groundingdino.util.inference import predict

def quat_to_rot(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]

    # Row 1
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Row 2
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Row 3
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def get_real_coords(x, y, camera_pos, camera_ori, depth_image, camera_intrinsic_mat, camera_rel_pos):
    """
    This function uses the depth camera pixel values, camera intrinsic matrix, and camera position
    to translate pixel values to real world cooridnates
    """

    assert False # Do I need to regularize depth, or can I use depth_linear directly?

    # Calculate 3D coordinates in camera frame
    cx = camera_intrinsic_mat[0][2]
    cy = camera_intrinsic_mat[1][2]
    fx = camera_intrinsic_mat[0][0]
    fy = camera_intrinsic_mat[1][1]

    camera_coords_z = depth_image[x,y]

    camera_coords_x = (x-cx)*camera_coords_z/fx
    camera_coords_y = (y-cy)*camera_coords_z/fy

    # Translate 3D coordinates to global frame
    c_coord = np.array([camera_coords_x, camera_coords_y, camera_coords_z])

    # Set up rotation Matrix based off of camera location
    Rotation = quat_to_rot(camera_ori)

    # Set up translation vector based off of actual camera position 
    translation = camera_pos

    world_coords = Rotation*c_coord + translation

    return world_coords

def obj_detection(dino_model, obj_tp, state, feature):
    img = state['robot0:eyes_Camera_sensor_rgb']

    if feature is not None:
        TEXT_PROMPT = f'{obj_tp}'
    else:
        TEXT_PROMP = f'{obj_tp} with {feature}'

    BOX_THRESHOLD = 0.35
    TEXT_THRESHOLD = 0.25

    boxes, logits, _ = predict(
        model=dino_model,
        image=img,
        caption=TEXT_PROMPT,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD
    )

    return np.stack(boxes, logits)


def get_vox_preds(camera_pos, camera_ori, belief, obj_tp, state, dino_model, feature=None):
    """
    Function to get predicted value of existence at each voxel (for an object type) 
    give observation

    :param:


    :returns: np.array with same size as belief, where voxels within observation are updated based on 
    predicted value of object existence.
    """

    voxel_preds = np.zeros_like(belief)

    # Get the set object bounding boxes and confidence scores for object types/features from state
    detected_objects = obj_detection(dino_model, obj_tp, state, feature)

    # Get the corresponding voxels
    for bbox, score in detected_objects:
        # Get bounding box center 
        # Grounding Dino format is cxcywh
        cx = bbox[0]
        cy = bbox[1]

        # Get xyz coordinates from image and depth
        x, y, z = get_real_coords(cx, cy, camera_pos, camera_ori, state['robot0:eyes_Camera_sensor_depth'], camera_intrinsic_mat, camera_rel_pos)

        # Translate to map coords and add to prediction
        xy = [x, y]
        map_resolution = belief.map_params['res']
        map_size = belief.map_params['size']
        vxy = world_to_map(xy, map_resolution, map_size)

        vx = vxy[0]
        vy = vxy[1]
        vz = int(z / map_resolution)

        # Put score in prediction output
        voxel_preds[vx, vy, vz] = score

    return voxel_preds
from geometry_msgs.msg import Pose
from api_moveit_clients.transform import Affine
from geometry_msgs.msg import Quaternion, Point


def affine_to_pose(affine: Affine) -> Pose:
    """
    TODO docstring

    Parameters
    ----------
    affine : Affine
        DESCRIPTION.

    Returns
    -------
    PoseMsg
        DESCRIPTION.

    """
    t = affine.translation
    r = affine.quat
    msg = Pose(
        position=Point(x=float(t[0]), y=float(t[1]), z=float(t[2])),
        orientation=Quaternion(x=float(r[0]), y=float(r[1]), z=float(r[2]), w=float(r[3])))
    return msg


def pose_to_affine(pose: Pose) -> Affine:
    """
    TODO docstring

    Parameters
    ----------
    pose : PoseMsg
        DESCRIPTION.

    Returns
    -------
    Affine
        DESCRIPTION.
    """
    affine = Affine(
        translation=[pose.position.x, pose.position.y, pose.position.z],
        rotation=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )
    return affine
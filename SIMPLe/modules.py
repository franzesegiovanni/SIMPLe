from ILoSA import ILoSA
from GILoSA import Transport
from GILoSA.tag_detector import Tag_Detector
from GILoSA.surface_pointcloud_detector import Surface_PointCloud_Detector
from GILoSA.read_pose_arm import LeftArmPose
from SIMPLe import SIMPLe
import rospy


class GILoSA_surface(Transport,Surface_PointCloud_Detector, SIMPLe):
    def __init__(self):
        rospy.init_node('GILoSA', anonymous=True)
        rospy.sleep(2)
        super(GILoSA_surface,self).__init__()
              
        
class GILoSA_tag(Transport, Tag_Detector, SIMPLe):
    def __init__(self):
        rospy.init_node('GILoSA', anonymous=True)
        rospy.sleep(2)
        super(GILoSA_tag,self).__init__()
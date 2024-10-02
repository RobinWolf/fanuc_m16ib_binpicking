class AI_Base:
    def __init__(self, score, pred_class, mask, bbox, area):
        
        self.score = score
        self.pred_class = pred_class
        self.mask = mask
        self.bbox = bbox
        self.area = area

class PCD_Base:
    def __init__(self, points, normals, rgb):

        self.points = points
        self.normals = normals
        self.rgb = rgb

class FPFH_Base:
    def __init__(self, pcd_downsampled , fpfh):

        self.pcd_downsampled=pcd_downsampled
        self.fpfh=fpfh
        

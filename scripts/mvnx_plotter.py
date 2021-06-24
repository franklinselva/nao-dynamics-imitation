from mvnx_parser import MVNX


class mvnx_plotter():
    def __init__(self):
        self.mvnx = MVNX()
        _, self.segments, self.sensors, self.joints, self.frames = self.mvnx.parse_all()
    